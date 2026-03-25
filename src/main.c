#include <version.h>
#include <stdint.h>

#include <errno.h>
#include <string.h>

#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/posix/time.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <vermin_collector_ros_msgs/msg/command.h>
#include <vermin_collector_ros_msgs/msg/feedback.h>


#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>

#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>

#define TIMER_FREQ 40000000 // 40MHz
#define FREQ_FROM_INTV(intv_ns) (1000000000.0 / intv_ns)
#define INTV_FROM_FREQ(freq_hz) (1000000000.0 / freq_hz)

#define STRIP_NODE DT_ALIAS(led_strip)
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);


// define stepper devices, related semaphores and idx as structs
typedef struct {
    const struct device *dev;
    struct k_sem *sem;
    uint8_t idx;
	const char * name;
} stepper_ctx_t;

stepper_ctx_t steppers[3]; // will be pitch, yaw, slide
#define PITCH_IDX 0
#define YAW_IDX 1
#define SLIDE_IDX 2

K_SEM_DEFINE(pitch_steps_completed_sem, 1, 1); // first 1 allows to enter semaphore right from the beginning
K_SEM_DEFINE(yaw_steps_completed_sem, 1, 1);
K_SEM_DEFINE(slide_steps_completed_sem, 1, 1);

void init_steppers(void)
{
    steppers[PITCH_IDX] = (stepper_ctx_t){
        .dev = DEVICE_DT_GET(DT_ALIAS(pitch_stepper)),
        .sem = &pitch_steps_completed_sem,
        .idx = PITCH_IDX,
		.name = "PITCH"
    };

    steppers[YAW_IDX] = (stepper_ctx_t){
        .dev = DEVICE_DT_GET(DT_ALIAS(yaw_stepper)),
        .sem = &yaw_steps_completed_sem,
        .idx = YAW_IDX,
		.name = "YAW"
    };

    steppers[SLIDE_IDX] = (stepper_ctx_t){
        .dev = DEVICE_DT_GET(DT_ALIAS(slide_stepper)),
        .sem = &slide_steps_completed_sem,
        .idx = SLIDE_IDX,
		.name = "SLIDE"
    };
}


// publisher & its variables: initilization
rcl_publisher_t feedback_pub;
vermin_collector_ros_msgs__msg__Feedback feedback_msg;

uint8_t current_state = vermin_collector_ros_msgs__msg__Feedback__CONFIGURING; // synchronous for all steppers
int32_t current_steps[3] = {0, 0, 0};
uint32_t current_frequency = 1000; // 1 kHz, sychronous for all steppers
enum stepper_micro_step_resolution current_res = STEPPER_MICRO_STEP_16; // synchronous for all steppers


// subscriber & its variables: initialization
rcl_subscription_t command_sub;
vermin_collector_ros_msgs__msg__Command command_msg;


#define RCCHECK(fn)                                                                      \
	{                                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK))                                                     \
		{                                                                                \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			for (;;)                                                                     \
			{                                                                            \
			};                                                                           \
		}                                                                                \
	}
#define RCSOFTCHECK(fn)                                                                    \
	{                                                                                      \
		rcl_ret_t temp_rc = fn;                                                            \
		if ((temp_rc != RCL_RET_OK))                                                       \
		{                                                                                  \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                  \
	}


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{	
		// get current data
		// frequency is updated in Setup routine as there is no get-method
		// state is updated at start and end of each routine	
		stepper_get_micro_step_res(steppers[PITCH_IDX].dev, &current_res);
		stepper_get_actual_position(steppers[PITCH_IDX].dev, &current_steps[PITCH_IDX]);
		stepper_get_actual_position(steppers[YAW_IDX].dev, &current_steps[YAW_IDX]);
		stepper_get_actual_position(steppers[SLIDE_IDX].dev, &current_steps[SLIDE_IDX]);
		
		// assign data to message
		feedback_msg.state = current_state;
		feedback_msg.current_steps[PITCH_IDX] = current_steps[PITCH_IDX];
		feedback_msg.current_steps[YAW_IDX] = current_steps[YAW_IDX];
		feedback_msg.current_steps[SLIDE_IDX] = current_steps[SLIDE_IDX];
		feedback_msg.frequency = current_frequency;
		feedback_msg.resolution = (uint8_t)current_res;

		RCSOFTCHECK(rcl_publish(&feedback_pub, &feedback_msg, NULL));
	}
}


void command_callback(const void *msgin)
{	// handles subscription to new incoming command
	const vermin_collector_ros_msgs__msg__Command *msg = msgin;
	LOG_INF("Received Command of type: %d\n0..SETUP, 1..TARGET, 2..HOMING\n", msg->command_type);

	k_sem_take(steppers[PITCH_IDX].sem, K_FOREVER);
	stepper_move_by(steppers[PITCH_IDX].dev, 10);

	k_sem_take(steppers[YAW_IDX].sem, K_FOREVER);
	stepper_move_by(steppers[YAW_IDX].dev, 50);

	k_sem_take(steppers[SLIDE_IDX].sem, K_FOREVER);
	stepper_move_by(steppers[SLIDE_IDX].dev, 100);
}



void stepper_callback(const struct device *dev, const enum stepper_event event, void *user_data)
{
	stepper_ctx_t *ctx = (stepper_ctx_t *)user_data;

	switch (event)
	{
	case STEPPER_EVENT_STEPS_COMPLETED:
		k_sem_give(ctx->sem);
		printk("Steps completed on stepper: %s\n", ctx->name);

		break;
	default:
		break;
	}
}

int main(void)
{
	rmw_uros_set_custom_transport(
		MICRO_ROS_FRAMING_REQUIRED,
		(void *)&default_params,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	// RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	// workaround since reset disconnects serial port from ubuntu wsl, wait till its back
	rcl_ret_t ret;
	do {
    	ret = rclc_support_init(&support, 0, NULL, &allocator);
    	if (ret != RCL_RET_OK) {
        	printk("Waiting for micro-ROS agent...\n");
        	k_sleep(K_MSEC(500));
    	}
	} while (ret != RCL_RET_OK);

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "zephyr_esp32_stepper_manager", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&feedback_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(vermin_collector_ros_msgs, msg, Feedback),
		"ESP32_Feedback"));

	// create timer for feedback publication
	rcl_timer_t timer;
	const uint32_t timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));


	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&command_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(vermin_collector_ros_msgs, msg, Command),
		"ESP32_Command"));

	// create executor
	rclc_executor_t executor;
	// 2 is number of handles = subscriptions + timers
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&command_sub,
		&command_msg,
		command_callback,
		ON_NEW_DATA));

	// init led-strip
	if (device_is_ready(strip))
	{
		LOG_INF("Found LED strip device %s", strip->name);
	}
	struct led_rgb color[] = {{0, 1, 0}};
	int rc = led_strip_update_rgb(strip, color, 1);
	if (rc)
	{
		LOG_ERR("couldn't update strip: %d", rc);
	}


	// check stepper devices
	init_steppers();
	for (uint8_t stepper_idx = 0; stepper_idx < 3; stepper_idx++){
		if (!device_is_ready(steppers[stepper_idx].dev))
		{
			printk("Device %s is not ready\n", steppers[stepper_idx].name);
			return -ENODEV;
		}
		printk("stepper is %p, name is %s\n", steppers[stepper_idx].dev, steppers[stepper_idx].name);
	
		stepper_set_event_callback(steppers[stepper_idx].dev, stepper_callback, &steppers[stepper_idx]);
		stepper_set_reference_position(steppers[stepper_idx].dev, 0);
		stepper_set_micro_step_res(steppers[stepper_idx].dev, current_res);
		stepper_set_microstep_interval(steppers[stepper_idx].dev, INTV_FROM_FREQ(current_frequency));
		stepper_enable(steppers[stepper_idx].dev);
	
	}


	while (1)
	{
		rclc_executor_spin_some(&executor, 100);
		usleep(100000);
	}

	// free resources
	RCCHECK(rclc_executor_fini(&executor))
	RCCHECK(rcl_publisher_fini(&feedback_pub, &node))
	RCCHECK(rcl_subscription_fini(&command_sub, &node))
	RCCHECK(rcl_timer_fini(&timer))
	RCCHECK(rcl_node_fini(&node))
	RCCHECK(rclc_support_fini(&support))

	return 0;
}