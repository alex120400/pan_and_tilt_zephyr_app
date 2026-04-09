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

/* #################################### system preparation steps #################################### */
typedef enum {
    STATE_READY,
    STATE_CONFIGURING,
    STATE_MOVING,
    STATE_CALIBRATING
} system_state_t;

volatile system_state_t system_state = STATE_CONFIGURING; // mirrors stepper state, synchronous for all steppers
volatile bool command_pending = false;


/* #################################### uROS preparation steps #################################### */
#define RCCHECK(fn) {                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK)) {                                                   \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			while(1);                                                                    \
		}																				 \
	}
#define RCSOFTCHECK(fn) {                                                                                           \
		rcl_ret_t temp_rc = fn;                                                                                     \
		if ((temp_rc != RCL_RET_OK)) printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
	}

// publisher & its variables
rcl_publisher_t feedback_pub;
vermin_collector_ros_msgs__msg__Feedback feedback_msg;

int32_t current_steps[3] = {0, 0, 0};
uint32_t current_frequency = 1000; // 1 kHz, sychronous for all steppers
enum stepper_micro_step_resolution current_res = STEPPER_MICRO_STEP_16; // synchronous for all steppers

// subscriber & its variables
rcl_subscription_t command_sub;
vermin_collector_ros_msgs__msg__Command active_command_msg;



/* #################################### Stepper preparation steps #################################### */
// #define TIMER_FREQ 40000000 // 40MHz
#define FREQ_FROM_INTV(intv_ns) (1000000000.0 / intv_ns)
#define INTV_FROM_FREQ(freq_hz) (1000000000.0 / freq_hz)

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

K_SEM_DEFINE(pitch_steps_completed_sem, 0, 1); // 0 means semaphore needs to be released before it may be taken for the first time
K_SEM_DEFINE(yaw_steps_completed_sem, 0, 1);
K_SEM_DEFINE(slide_steps_completed_sem, 0, 1);



/* #################################### LED-strip preparation steps #################################### */
#define STRIP_NODE DT_ALIAS(led_strip)
#define LED_BRIGHTNESS 20
#define STRIP_NUM_PIXELS 1
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }
#define READY_IDX 0
#define CONFIGURING_IDX 1
#define MOVING_IDX 2
#define CALIBRATING_IDX 3
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

static struct led_rgb colors[] = {
	RGB(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS), /* white */
	RGB(LED_BRIGHTNESS, 0x00, 0x00), /* red */
	RGB(0x00, LED_BRIGHTNESS, 0x00), /* green */
	RGB(0x00, 0x00, LED_BRIGHTNESS), /* blue */
};


/* #################################### External LED & Button for Tests preparation step #################################### */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(my_led), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(my_button), gpios);
static struct gpio_callback btn_cb_data;

// GPIO callback (ISR)
void button_isr(const struct device *dev,
                struct gpio_callback *cb,
                uint32_t pins)
{
    // Check if the correct button was pressed
    if (BIT(btn.pin) & pins) {
        gpio_pin_toggle_dt(&led);
    }
}




/* #################################### Callback definitions #################################### */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time){ // used to publish Feedback regularly
	RCLC_UNUSED(last_call_time);
	if (timer != NULL){	
		// frequency & resolution are updated in Setup routine
		// state is updated at start and end of each routine (Target, Setup, Homeing)	
		stepper_get_actual_position(steppers[PITCH_IDX].dev, &current_steps[PITCH_IDX]);
		stepper_get_actual_position(steppers[YAW_IDX].dev, &current_steps[YAW_IDX]);
		stepper_get_actual_position(steppers[SLIDE_IDX].dev, &current_steps[SLIDE_IDX]);
		
		// assign data to message
		feedback_msg.state = (uint8_t)system_state;
		feedback_msg.current_steps[PITCH_IDX] = current_steps[PITCH_IDX];
		feedback_msg.current_steps[YAW_IDX] = current_steps[YAW_IDX];
		feedback_msg.current_steps[SLIDE_IDX] = current_steps[SLIDE_IDX];
		feedback_msg.frequency = current_frequency;
		feedback_msg.resolution = (uint8_t)current_res;

		RCSOFTCHECK(rcl_publish(&feedback_pub, &feedback_msg, NULL));
	}
}

void command_callback(const void *msgin){	// handles subscription to new incoming command
	const vermin_collector_ros_msgs__msg__Command *msg = msgin;
	LOG_INF("Received Command of type: %d\n0..SETUP, 1..TARGET, 2..HOMING\n", msg->command_type);

	// active_command_msg = *msg; // redundant as active_command_msg is provided as buffer in the add_subscription handling
	command_pending = true; // signal main loop

	// k_sem_take(steppers[PITCH_IDX].sem, K_FOREVER);
	// stepper_move_by(steppers[PITCH_IDX].dev, 10);

	// k_sem_take(steppers[YAW_IDX].sem, K_FOREVER);
	// stepper_move_by(steppers[YAW_IDX].dev, 50);

	// k_sem_take(steppers[SLIDE_IDX].sem, K_FOREVER);
	// stepper_move_by(steppers[SLIDE_IDX].dev, 100);
}

void stepper_callback(const struct device *dev, const enum stepper_event event, void *user_data){
	stepper_ctx_t *ctx = (stepper_ctx_t *)user_data;
	LOG_INF("Event %d occured on stepper: %s\n", event, ctx->name);
	switch (event){
	case STEPPER_EVENT_STEPS_COMPLETED:
		k_sem_give(ctx->sem);
		LOG_INF("Steps completed on stepper: %s\n", ctx->name);
		break;
	default:
		break;
	}
}



/* #################################### device init functions #################################### */
void init_led(void){
	if (!device_is_ready(strip)){
		LOG_ERR("Device %s is not ready\n", strip->name);
		return;
	}
	LOG_INF("Found LED strip device %s", strip->name);
	if (led_strip_update_rgb(strip, &colors[CONFIGURING_IDX], STRIP_NUM_PIXELS) != 0) {
    LOG_ERR("LED update failed");
	}
}

void init_steppers(void){
	// fill steppers list
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

	// check devices and set default values
	for (uint8_t stepper_idx = 0; stepper_idx < 3; stepper_idx++){
		if (!device_is_ready(steppers[stepper_idx].dev)){
			LOG_ERR("Device %s is not ready\n", steppers[stepper_idx].name);
			return;
		}
		LOG_INF("stepper is %p, name is %s\n", steppers[stepper_idx].dev, steppers[stepper_idx].name);
	
		stepper_set_event_callback(steppers[stepper_idx].dev, stepper_callback, &steppers[stepper_idx]);
		stepper_set_reference_position(steppers[stepper_idx].dev, 0);
		stepper_set_micro_step_res(steppers[stepper_idx].dev, current_res);
		stepper_set_microstep_interval(steppers[stepper_idx].dev, INTV_FROM_FREQ(current_frequency));
		stepper_enable(steppers[stepper_idx].dev);	
	}
}



/* #################################### command routines and helper functions #################################### */
void handle_setup(void){
    LOG_INF("SETUP: configuring...\n");
    if (led_strip_update_rgb(strip, &colors[CONFIGURING_IDX], STRIP_NUM_PIXELS) != 0) {
    LOG_ERR("LED update failed");
	}
	system_state = STATE_CONFIGURING; // change system state
	command_pending = false;

	current_frequency = active_command_msg.frequency;
	switch(active_command_msg.resolution){
		case 8:
			current_res = STEPPER_MICRO_STEP_8;
			break;
		case 16:
			current_res = STEPPER_MICRO_STEP_16;
			break;
		case 32:
			current_res = STEPPER_MICRO_STEP_32;
			break;
		case 64:
			current_res = STEPPER_MICRO_STEP_64;
			break;
		default:
			current_res = STEPPER_MICRO_STEP_16;
			break;
	}

	// update stepper values
	for (uint8_t stepper_idx = 0; stepper_idx < 3; stepper_idx++){
		stepper_set_micro_step_res(steppers[stepper_idx].dev, current_res);
		stepper_set_microstep_interval(steppers[stepper_idx].dev, INTV_FROM_FREQ(current_frequency));
	}

    LOG_INF("SETUP done\n");

    system_state = STATE_READY;
	if (led_strip_update_rgb(strip, &colors[READY_IDX], STRIP_NUM_PIXELS) != 0) {
    LOG_ERR("LED update failed");
	}
}

void handle_homing(void){
    system_state = STATE_CALIBRATING;
	command_pending = false;
	if (led_strip_update_rgb(strip, &colors[CALIBRATING_IDX], STRIP_NUM_PIXELS) != 0) {
    LOG_ERR("LED update failed");
	}

    LOG_INF("HOMING: calibrating...\n");

    k_sleep(K_SECONDS(3));

    LOG_INF("HOMING done\n");

    system_state = STATE_READY;
	if (led_strip_update_rgb(strip, &colors[READY_IDX], STRIP_NUM_PIXELS) != 0) {
    LOG_ERR("LED update failed");
	}
}

void simple_target(vermin_collector_ros_msgs__msg__Command *cmd){
    LOG_INF("Simple target\n");

    move_and_wait(&steppers[PITCH_IDX], cmd->step_goals[PITCH_IDX]);
    move_and_wait(&steppers[YAW_IDX],   cmd->step_goals[YAW_IDX]);
    move_and_wait(&steppers[SLIDE_IDX], cmd->step_goals[SLIDE_IDX]);
}

void move_and_wait(stepper_ctx_t *ctx, int32_t steps){
	k_sem_reset(ctx->sem); // make sure semaphore is 0 right now
	LOG_INF("Moving stepper: %s\n", ctx->name);
    stepper_move_by(ctx->dev, steps);
    k_sem_take(ctx->sem, K_FOREVER);   // take semaphore and wait till movement is finished, this will block here since semaphore are 0-initialized
}

void star_pattern(vermin_collector_ros_msgs__msg__Command *cmd){
	/* Star pattern looks like:
			\   |   /
			 \  |  /
			  \ | /
		-----------------
		  	  / | \
		     /  |  \
			/   |   \
	movement may be interpreted as pitch: up&down
								   yaw:   left&right
		=> horizontal and vertical only one stepper, 
		=> diagonals both (one diag both in same direction, at the other opposite directions)
	*/
	
    int32_t d = cmd->star_diameter;

    LOG_INF("Star pattern, diameter: %d\n", d);

    // move slide to goal first
    move_and_wait(&steppers[SLIDE_IDX], cmd->step_goals[SLIDE_IDX]);

    // ---- PITCH only ----
    move_and_wait(&steppers[PITCH_IDX],  d);
    move_and_wait(&steppers[PITCH_IDX], -2*d);
    move_and_wait(&steppers[PITCH_IDX],  d);

    // ---- YAW only ----
    move_and_wait(&steppers[YAW_IDX],  d);
    move_and_wait(&steppers[YAW_IDX], -2*d);
    move_and_wait(&steppers[YAW_IDX],  d);

    // ---- BOTH in the same direction ----

    stepper_move_by(steppers[PITCH_IDX].dev,  d);
    stepper_move_by(steppers[YAW_IDX].dev,    d);
    k_sem_take(steppers[PITCH_IDX].sem, K_FOREVER);
    k_sem_take(steppers[YAW_IDX].sem,   K_FOREVER);

    stepper_move_by(steppers[PITCH_IDX].dev, -2*d);
    stepper_move_by(steppers[YAW_IDX].dev,   -2*d);
    k_sem_take(steppers[PITCH_IDX].sem, K_FOREVER);
    k_sem_take(steppers[YAW_IDX].sem,   K_FOREVER);

    stepper_move_by(steppers[PITCH_IDX].dev,  d);
    stepper_move_by(steppers[YAW_IDX].dev,    d);
    k_sem_take(steppers[PITCH_IDX].sem, K_FOREVER);
    k_sem_take(steppers[YAW_IDX].sem,   K_FOREVER);

    // ---- BOTH in the opposite direction ----
    stepper_move_by(steppers[PITCH_IDX].dev, d);
    stepper_move_by(steppers[YAW_IDX].dev, -d);
    k_sem_take(steppers[PITCH_IDX].sem, K_FOREVER);
    k_sem_take(steppers[YAW_IDX].sem,   K_FOREVER);

    stepper_move_by(steppers[PITCH_IDX].dev, -2*d);
    stepper_move_by(steppers[YAW_IDX].dev, 2*d);
    k_sem_take(steppers[PITCH_IDX].sem, K_FOREVER);
    k_sem_take(steppers[YAW_IDX].sem,   K_FOREVER);

    stepper_move_by(steppers[PITCH_IDX].dev, d);
    stepper_move_by(steppers[YAW_IDX].dev, -d);
    k_sem_take(steppers[PITCH_IDX].sem, K_FOREVER);
    k_sem_take(steppers[YAW_IDX].sem,   K_FOREVER);
}

void handle_target(vermin_collector_ros_msgs__msg__Command *cmd){
	vermin_collector_ros_msgs__msg__Command local_cmd = *cmd; // copy command locally, sothat new one may arrive
	command_pending = false;
    system_state = STATE_MOVING;
	if (led_strip_update_rgb(strip, &colors[MOVING_IDX], STRIP_NUM_PIXELS) != 0) {
    LOG_ERR("LED update failed");
	}

    if (local_cmd.star_diameter == 0) {
        simple_target(&local_cmd);
    } else {
        star_pattern(&local_cmd);
    }

    system_state = STATE_READY;
	if (led_strip_update_rgb(strip, &colors[READY_IDX], STRIP_NUM_PIXELS) != 0) {
    LOG_ERR("LED update failed");
	}
}



/* #################################### main loop #################################### */
int main(void){
	// init peripherial devices
	init_led();
	init_steppers();


	// external peripherals for testing
	int ret_led, ret_btn;
	if ((!gpio_is_ready_dt(&led)) || (!gpio_is_ready_dt(&btn))) {
        printk("ERROR: blue led or button not ready\r\n");
		return 0;
	}

	ret_led = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	ret_btn = gpio_pin_configure_dt(&btn, GPIO_INPUT);
    if ((ret_led < 0) || (ret_btn<0)) {
        printk("ERROR: could not set button as input or led as output\r\n");
        return 0;
    }

    ret_btn = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE); // Configure the interrupt
    if (ret_btn < 0) {
        printk("ERROR: could not configure button as interrupt source\r\n");
        return 0;
    }

    gpio_init_callback(&btn_cb_data, button_isr, BIT(btn.pin)); // Connect callback function (ISR) to interrupt source
    gpio_add_callback(btn.port, &btn_cb_data);



	// steup serial transport
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
	// RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); -> fails with ubuntu wsl
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
		&active_command_msg,
		command_callback,
		ON_NEW_DATA));


	// now ready for taking commands
	system_state = STATE_READY;
	if (led_strip_update_rgb(strip, &colors[READY_IDX], STRIP_NUM_PIXELS) != 0) {
    LOG_ERR("LED update failed");
	}

	

	while (1){
		rclc_executor_spin_some(&executor, 100);
		if (command_pending) {
        // command_pending = false; // done in routines

        switch (active_command_msg.command_type)
        {
        case vermin_collector_ros_msgs__msg__Command__SETUP:
            handle_setup();
            break;

        case vermin_collector_ros_msgs__msg__Command__TARGET:
            handle_target(&active_command_msg);
            break;

        case vermin_collector_ros_msgs__msg__Command__HOMING:
            handle_homing();
            break;

        default:
            printk("Unknown command\n");
            break;
        }
    }

    k_msleep(100);
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