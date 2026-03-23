#include <version.h>

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
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>


#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>

#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>

#define STRIP_NODE DT_ALIAS(led_strip)
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

const struct device *stepper = DEVICE_DT_GET(DT_ALIAS(stepper));

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

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		msg.data++;
	}
}

rcl_subscription_t subscription;
std_msgs__msg__Int32 sub_msg;

void sub_callback(const void *msgin)
{
	const std_msgs__msg__Int32 *msg = msgin;
	LOG_INF("Received msg: %d", msg->data);
}

K_SEM_DEFINE(steps_completed_sem, 0, 1);

void stepper_callback(const struct device *dev, const enum stepper_event event, void *user_data)
{
	switch (event)
	{
	case STEPPER_EVENT_STEPS_COMPLETED:
		k_sem_give(&steps_completed_sem);
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
	RCCHECK(rclc_node_init_default(&node, "zephyr_int32_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"zephyr_int32_publisher"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	RCCHECK(rclc_subscription_init_default(
		&subscription,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"zephyr_int32_subscription"));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&subscription,
		&sub_msg,
		sub_callback,
		ON_NEW_DATA));

	msg.data = 0;

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

	if (!device_is_ready(stepper))
	{
		printk("Device %s is not ready\n", stepper->name);
		return -ENODEV;
	}
	printk("stepper is %p, name is %s\n", stepper, stepper->name);

	stepper_set_microstep_interval(stepper, 1000000);
	stepper_set_event_callback(stepper, stepper_callback, NULL);
	stepper_enable(stepper);
	stepper_set_reference_position(stepper, 0);
	stepper_move_by(stepper, 1000);

	while (1)
	{
		rclc_executor_spin_some(&executor, 100);
		usleep(100000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))
	return 0;
}