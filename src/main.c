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

#define READY_IDX 0
#define CONFIGURING_IDX 1
#define MOVING_IDX 2
#define CALIBRATING_IDX 3

volatile system_state_t system_state = STATE_CONFIGURING; // mirrors stepper state, synchronous for all steppers
volatile bool command_pending = false;

#define PITCH_STEPS_PER_REV 610
#define YAW_STEPS_PER_REV 1694

#define COMMAND_STACK_SIZE 4096
K_THREAD_STACK_DEFINE(command_stack, COMMAND_STACK_SIZE); // define stack areas for the threads
static struct k_thread command_thread; // define thread data structs


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
uint16_t current_frequencies[3] = {100, 100, 100}; // 100 Hz initially for all steppers
enum stepper_micro_step_resolution current_res = STEPPER_MICRO_STEP_16; // synchronous for all steppers

// subscriber & its variables
rcl_subscription_t command_sub;
vermin_collector_ros_msgs__msg__Command active_command_msg;

typedef enum {
    simple_homing,
    soft_homing,
    hard_homing
} homing_flag_t;


/* #################################### Stepper preparation steps #################################### */
#define FREQ_FROM_INTV(intv_ns) (1000000000.0 / intv_ns)
#define INTV_FROM_FREQ(freq_hz) (1000000000.0 / freq_hz)

// define stepper devices, related semaphores and idx as structs
typedef enum {
	STEPPER_DISABLED,
    STEPPER_ENABLED,
    STEPPER_MOVING
} stepper_state_t;

typedef struct {
    const struct device *dev;
    // struct k_sem *sem;
    uint8_t idx;
	const char * name;
	int32_t magn_pos_hl;
	int32_t magn_pos_lh;
	stepper_state_t state;

} stepper_ctx_t;

stepper_ctx_t steppers[3]; // will be pitch, yaw, slide
#define PITCH_IDX 0
#define YAW_IDX 1
#define SLIDE_IDX 2


/* #################################### LED-strip preparation steps #################################### */
#define STRIP_NODE DT_ALIAS(led_strip)
#define LED_BRIGHTNESS 20
#define STRIP_NUM_PIXELS 1
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

static struct led_rgb colors[] = {
	RGB(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS), /* white */
	RGB(LED_BRIGHTNESS, 0x00, 0x00), /* red */
	RGB(0x00, LED_BRIGHTNESS, 0x00), /* green */
	RGB(0x00, 0x00, LED_BRIGHTNESS), /* blue */
};


/* #################################### Laser LED preparation step #################################### */
static const struct gpio_dt_spec laser_anode = GPIO_DT_SPEC_GET(DT_ALIAS(laser_led_anode), gpios);
static const struct gpio_dt_spec laser_kathode = GPIO_DT_SPEC_GET(DT_ALIAS(laser_led_kathode), gpios);


/* #################################### Hall Sensor preparation step #################################### */
static const struct gpio_dt_spec pitch_hall = GPIO_DT_SPEC_GET(DT_ALIAS(pitch_hall), gpios);
static struct gpio_callback pitch_hall_cb_data;
static const struct gpio_dt_spec yaw_hall = GPIO_DT_SPEC_GET(DT_ALIAS(yaw_hall), gpios);
static struct gpio_callback yaw_hall_cb_data;
static const struct gpio_dt_spec slide_hall = GPIO_DT_SPEC_GET(DT_ALIAS(slide_hall), gpios);
static struct gpio_callback slide_hall_cb_data;

// Hall sensor callbacks [GPIO callbacks/ISRs]
void hall_isr(const struct device *dev,
                struct gpio_callback *cb,
                uint32_t pins)
{
    // Check if the correct button was pressed
    if (BIT(pitch_hall.pin) & pins) {
		if (gpio_pin_get(pitch_hall.port, pitch_hall.pin) == 0x0) {// high -> low, are hovering over magnet
			stepper_get_actual_position(steppers[PITCH_IDX].dev, &steppers[PITCH_IDX].magn_pos_hl);
		} else { // low -> high, just left magnet
			stepper_get_actual_position(steppers[PITCH_IDX].dev, &steppers[PITCH_IDX].magn_pos_lh);
		}
	}
	else if (BIT(yaw_hall.pin) & pins) {
		if (gpio_pin_get(yaw_hall.port, yaw_hall.pin) == 0x0) {// high -> low, are hovering over magnet
			stepper_get_actual_position(steppers[YAW_IDX].dev, &steppers[YAW_IDX].magn_pos_hl);
		} else { // low -> high, just left magnet
			stepper_get_actual_position(steppers[YAW_IDX].dev, &steppers[YAW_IDX].magn_pos_lh);
		}
	}
	else if (BIT(slide_hall.pin) & pins) {
		if (gpio_pin_get(slide_hall.port, slide_hall.pin) == 0x0){// high -> low, are hovering over magnet
			stepper_get_actual_position(steppers[SLIDE_IDX].dev, &steppers[SLIDE_IDX].magn_pos_hl);
		} else { // low -> high, just left magnet
			stepper_get_actual_position(steppers[SLIDE_IDX].dev, &steppers[SLIDE_IDX].magn_pos_lh);
    	}
	}
	else LOG_ERR("Weird pin called ISR!");
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
		feedback_msg.resolution = (uint8_t)current_res;
		for (uint8_t loc_step_idx=0; loc_step_idx<3;loc_step_idx++){
			feedback_msg.current_steps[loc_step_idx] = current_steps[loc_step_idx];
			feedback_msg.en_motors[loc_step_idx] = steppers[loc_step_idx].state;
			feedback_msg.frequencies[loc_step_idx] = current_frequencies[loc_step_idx];
		}

		RCSOFTCHECK(rcl_publish(&feedback_pub, &feedback_msg, NULL));
	}
}

void command_callback(const void *msgin){	// handles subscription to new incoming command
	const vermin_collector_ros_msgs__msg__Command *msg = msgin;
	LOG_INF("Received Command of type: %d\n0..SETUP, 1..TARGET, 2..HOMING, 3..SOFT_HOMING, 4..HARD_HOMING\n", msg->command_type);

	// active_command_msg = *msg; // redundant as active_command_msg is provided as buffer in the add_subscription handling
	if (system_state == STATE_READY){
		command_pending = true; // signal for command thread to act
	} else {
		LOG_INF("Ignored Command of type: %d\nSystem state is %d\n1..CONFIGURING, 2..MOVING, 3..CALIBRATING -> wait for 0..READY\n", msg->command_type);
	}
	
}

void stepper_callback(const struct device *dev, const enum stepper_event event, void *user_data){
	stepper_ctx_t *ctx = (stepper_ctx_t *)user_data;
	LOG_DBG("Event %d occured on stepper: %s\n", event, ctx->name);
	switch (event){
	case STEPPER_EVENT_STEPS_COMPLETED:
		ctx->state = STEPPER_ENABLED;
		LOG_DBG("Steps completed on stepper: %s\n", ctx->name);
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

void init_laser(void){
	int ret;

	ret = gpio_is_ready_dt(&laser_anode) && gpio_is_ready_dt(&laser_kathode);
	if (!ret) {
		LOG_ERR("Laser LED gpios not ready!");
		return;
	}

	ret = gpio_pin_configure_dt(&laser_anode, GPIO_OUTPUT_INACTIVE); // is active high
	if (ret < 0) {
		LOG_ERR("Laser LED anode init failed!");
		return;
	}

	ret = gpio_pin_configure_dt(&laser_kathode, GPIO_OUTPUT_ACTIVE); // is active low
	if (ret < 0) {
		LOG_ERR("Laser LED kathode init failed!");
		return;
	}
	return;
}

void init_steppers(void){
	// fill steppers list
    steppers[PITCH_IDX] = (stepper_ctx_t){
        .dev = DEVICE_DT_GET(DT_ALIAS(pitch_stepper)),
        // .sem = &pitch_steps_completed_sem,
        .idx = PITCH_IDX,
		.name = "PITCH",
		.magn_pos_hl = 0x0000,
		.magn_pos_lh = 0x0000,
		.state = STEPPER_DISABLED
    };

    steppers[YAW_IDX] = (stepper_ctx_t){
        .dev = DEVICE_DT_GET(DT_ALIAS(yaw_stepper)),
        // .sem = &yaw_steps_completed_sem,
        .idx = YAW_IDX,
		.name = "YAW",
		.magn_pos_hl = 0x0000,
		.magn_pos_lh = 0x0000,
		.state = STEPPER_DISABLED
    };

    steppers[SLIDE_IDX] = (stepper_ctx_t){
        .dev = DEVICE_DT_GET(DT_ALIAS(slide_stepper)),
        // .sem = &slide_steps_completed_sem,
        .idx = SLIDE_IDX,
		.name = "SLIDE",
		.magn_pos_hl = 0x0000,
		.magn_pos_lh = 0x0000,
		.state = STEPPER_DISABLED
    };

	// check devices and set default values
	for (uint8_t stepper_idx = 0; stepper_idx < 3; stepper_idx++){
		if (!device_is_ready(steppers[stepper_idx].dev)){
			LOG_ERR("Device %s is not ready\n", steppers[stepper_idx].name);
			return;
		}
		LOG_DBG("stepper is %p, name is %s\n", steppers[stepper_idx].dev, steppers[stepper_idx].name);
	
		stepper_set_event_callback(steppers[stepper_idx].dev, stepper_callback, &steppers[stepper_idx]);
		stepper_set_reference_position(steppers[stepper_idx].dev, 0);
		stepper_set_micro_step_res(steppers[stepper_idx].dev, current_res);
		stepper_set_microstep_interval(steppers[stepper_idx].dev, INTV_FROM_FREQ(current_frequencies[stepper_idx]));
		stepper_enable(steppers[stepper_idx].dev);
		steppers[stepper_idx].state = STEPPER_ENABLED;
	}
}

void init_halls(void){
	if ((!gpio_is_ready_dt(&pitch_hall)) || (!gpio_is_ready_dt(&yaw_hall)) || (!gpio_is_ready_dt(&slide_hall))) {
        LOG_ERR("ERROR: One of the Hall sensors is not ready");
		return;
	}

	int ret;
	ret = gpio_pin_configure_dt(&pitch_hall, GPIO_INPUT);
	ret += gpio_pin_configure_dt(&yaw_hall, GPIO_INPUT);
	ret += gpio_pin_configure_dt(&slide_hall, GPIO_INPUT);

    if (ret < 0) {
        LOG_ERR("Could not set Hall-GPIOs as inputs");
        return;
    }

    ret = gpio_pin_interrupt_configure_dt(&pitch_hall, GPIO_INT_EDGE_BOTH); // Configure the interrupt
    ret += gpio_pin_interrupt_configure_dt(&yaw_hall, GPIO_INT_EDGE_BOTH); // Configure the interrupt
    ret += gpio_pin_interrupt_configure_dt(&slide_hall, GPIO_INT_EDGE_BOTH); // Configure the interrupt
    if (ret < 0) {
        LOG_ERR("Could not configure hall sensors as interrupt sources");
        return;
    }

    gpio_init_callback(&pitch_hall_cb_data, hall_isr, BIT(pitch_hall.pin)); // Connect callback function (ISR) to interrupt source
    gpio_init_callback(&yaw_hall_cb_data, hall_isr, BIT(yaw_hall.pin)); // Connect callback function (ISR) to interrupt source
    gpio_init_callback(&slide_hall_cb_data, hall_isr, BIT(slide_hall.pin)); // Connect callback function (ISR) to interrupt source
    gpio_add_callback(pitch_hall.port, &pitch_hall_cb_data);
	gpio_add_callback(yaw_hall.port, &yaw_hall_cb_data);
	gpio_add_callback(slide_hall.port, &slide_hall_cb_data);

	// ret = gpio_pin_interrupt_configure_dt(&pitch_hall, GPIO_INT_MODE_DISABLE_ONLY); // Disable interrupt for now
    // ret += gpio_pin_interrupt_configure_dt(&yaw_hall, GPIO_INT_MODE_DISABLE_ONLY); // Disable interrupt for now
    // ret += gpio_pin_interrupt_configure_dt(&slide_hall, GPIO_INT_MODE_DISABLE_ONLY); // Disable interrupt for now
    // if (ret < 0) {
    //     LOG_ERR("Could not disable hall sensors interrupts");
    //     return;
    // }
}


/* #################################### command routines and helper functions #################################### */
void handle_setup(void){
    LOG_DBG("SETUP: configuring...\n");
    if (led_strip_update_rgb(strip, &colors[CONFIGURING_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}
	system_state = STATE_CONFIGURING; // change system state
	command_pending = false;

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
		current_frequencies[stepper_idx] = active_command_msg.frequency_goals[stepper_idx];
		stepper_set_microstep_interval(steppers[stepper_idx].dev, INTV_FROM_FREQ(current_frequencies[stepper_idx]));
		if (active_command_msg.en_motors[stepper_idx]){
			stepper_enable(steppers[stepper_idx].dev);
			steppers[stepper_idx].state = STEPPER_ENABLED;
		}
		else {
			stepper_disable(steppers[stepper_idx].dev);
			steppers[stepper_idx].state = STEPPER_DISABLED;
		}
	}

    LOG_DBG("SETUP done\n");

    system_state = STATE_READY;
	if (led_strip_update_rgb(strip, &colors[READY_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}
}

void wait_for_movement_to_finish(stepper_ctx_t *ctx){
	while(ctx->state == STEPPER_MOVING){
		k_msleep(10);
	} // wait until movement is finished
}

void move_savely_to(stepper_ctx_t *ctx, int32_t step_goal){
	if (ctx->state == STEPPER_DISABLED) {
		LOG_ERR("Stepper %s not enabled, cannot move!", ctx->name);
		return;
	}

	wait_for_movement_to_finish(ctx);
	LOG_DBG("Moving stepper: %s\n", ctx->name);
	ctx->state = STEPPER_MOVING; // reset in stepper callback
    stepper_move_to(ctx->dev, step_goal);
}

void move_savely_by(stepper_ctx_t *ctx, int32_t step_amount){
	if (ctx->state == STEPPER_DISABLED) {
		LOG_ERR("Stepper %s not enabled, cannot move!", ctx->name);
		return;
	}

	wait_for_movement_to_finish(ctx);
	LOG_DBG("Moving stepper: %s\n", ctx->name);
	ctx->state = STEPPER_MOVING; // reset in stepper callback
    stepper_move_by(ctx->dev, step_amount);
}


void handle_homing(homing_flag_t homing_flag){
    system_state = STATE_CALIBRATING;
	command_pending = false;
	if (led_strip_update_rgb(strip, &colors[CALIBRATING_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}

	uint32_t home_goal = 0x00;
	// Pitch and Yaw homeing only for now
	// uint8_t ret = 0;
	// ret = gpio_pin_interrupt_configure_dt(&pitch_hall, GPIO_INT_MODE_ENABLE_ONLY); // Enable interrupt for now
    // ret += gpio_pin_interrupt_configure_dt(&yaw_hall, GPIO_INT_MODE_ENABLE_ONLY); // Enable interrupt for now
    // ret += gpio_pin_interrupt_configure_dt(&slide_hall, GPIO_INT_MODE_ENABLE_ONLY); // Enable interrupt for now
    // if (ret < 0) {
    //     LOG_ERR("Could not enable hall sensors interrupts");
    // }

	if (homing_flag == hard_homing){
		// reset positions
		steppers[PITCH_IDX].magn_pos_hl = 0x00;
		steppers[PITCH_IDX].magn_pos_hl = 0x00;
		steppers[YAW_IDX].magn_pos_hl = 0x00;
		steppers[YAW_IDX].magn_pos_hl = 0x00;

		// set current pos to zero to move in positive step range
		stepper_set_reference_position(steppers[PITCH_IDX].dev, 0); // new home position is reference 0
		stepper_set_reference_position(steppers[YAW_IDX].dev, 0); // new home position is reference 0

		// move one revolution and wait till its finished
		move_savely_by(&steppers[PITCH_IDX], PITCH_STEPS_PER_REV*current_res);
		move_savely_by(&steppers[YAW_IDX], YAW_STEPS_PER_REV*current_res);
		wait_for_movement_to_finish(&steppers[PITCH_IDX]);
		wait_for_movement_to_finish(&steppers[YAW_IDX]);

		// Hall ISRs should have fired by now and updated the magnetic positions
		if (steppers[PITCH_IDX].magn_pos_lh > steppers[PITCH_IDX].magn_pos_hl){ // general case, encounterd magnet along the way
			home_goal = (steppers[PITCH_IDX].magn_pos_lh + steppers[PITCH_IDX].magn_pos_hl) / 2;
		}
		else { // special case, started in magnetic region, ended with high-low transition, need to add a revolution to low-high tranistion
			home_goal = (steppers[PITCH_IDX].magn_pos_hl + (steppers[PITCH_IDX].magn_pos_lh + PITCH_STEPS_PER_REV*current_res)) / 2;
		}
		move_savely_to(&steppers[PITCH_IDX], home_goal);
		wait_for_movement_to_finish(&steppers[PITCH_IDX]); // wait untill home is reached
		stepper_set_reference_position(steppers[PITCH_IDX].dev, 0); // new home position is reference 0

		if (steppers[YAW_IDX].magn_pos_lh > steppers[YAW_IDX].magn_pos_hl){
			home_goal = (steppers[YAW_IDX].magn_pos_lh + steppers[YAW_IDX].magn_pos_hl) / 2;
		}
		else {
			home_goal = (steppers[YAW_IDX].magn_pos_hl + (steppers[YAW_IDX].magn_pos_lh + YAW_STEPS_PER_REV*current_res)) / 2;
		}
		move_savely_to(&steppers[YAW_IDX], home_goal);
		wait_for_movement_to_finish(&steppers[YAW_IDX]); // wait until home is reached
		stepper_set_reference_position(steppers[YAW_IDX].dev, 0); // new home position is reference 0
	} else if (homing_flag == soft_homing) {
		// set current position as home
		stepper_set_reference_position(steppers[PITCH_IDX].dev, 0); // new home position is reference 0
		stepper_set_reference_position(steppers[YAW_IDX].dev, 0); // new home position is reference 0
		stepper_set_reference_position(steppers[SLIDE_IDX].dev, 0); // new home position is reference 0
	} else { // homing, just move to last set 0 position
		move_savely_to(&steppers[PITCH_IDX], 0);
		move_savely_to(&steppers[YAW_IDX], 0);
		move_savely_to(&steppers[SLIDE_IDX], 0);
		wait_for_movement_to_finish(&steppers[PITCH_IDX]);
		wait_for_movement_to_finish(&steppers[YAW_IDX]);
		wait_for_movement_to_finish(&steppers[SLIDE_IDX]);
	}

	// ret = gpio_pin_interrupt_configure_dt(&pitch_hall, GPIO_INT_MODE_DISABLE_ONLY); // Disable interrupt again
    // ret += gpio_pin_interrupt_configure_dt(&yaw_hall, GPIO_INT_MODE_DISABLE_ONLY); // Disable interrupt again
    // ret += gpio_pin_interrupt_configure_dt(&slide_hall, GPIO_INT_MODE_DISABLE_ONLY); // Disable interrupt again
    // if (ret < 0) {
    //     LOG_ERR("Could not disable hall sensors interrupts");
    // }
	
    system_state = STATE_READY;
	if (led_strip_update_rgb(strip, &colors[READY_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}
}


void activate_led_for_ms_blocking(uint32_t duration_ms){
	if (duration_ms == 0) return; // skip this if duration is zero

	/* FORCE reconfiguration right before use */
    gpio_pin_configure_dt(&laser_anode, GPIO_OUTPUT); // do not change state yet
    gpio_pin_configure_dt(&laser_kathode, GPIO_OUTPUT_ACTIVE); // active low, make sure it is low here
	
	gpio_pin_set_dt(&laser_anode, 1); // logical set! anode is active high
	LOG_DBG("Laser activated for &d ms", duration_ms);
	k_msleep(duration_ms);
	gpio_pin_set_dt(&laser_anode, 0);
	return ;
}


void activate_led_non_blocking(){
	/* FORCE reconfiguration right before use */
    gpio_pin_configure_dt(&laser_anode, GPIO_OUTPUT_ACTIVE); // active high, activate led
    gpio_pin_configure_dt(&laser_kathode, GPIO_OUTPUT_ACTIVE); // active low, make sure it is low here
	return;
}


void deactivate_led_non_blocking(){
	/* FORCE reconfiguration right before use */
    gpio_pin_configure_dt(&laser_anode, GPIO_OUTPUT_INACTIVE); // active high, deactivate led
    gpio_pin_configure_dt(&laser_kathode, GPIO_OUTPUT_ACTIVE); // active low, make sure it is low here
	return;
}


void scan_pattern(uint32_t d){
	/* Scan pattern looks like:
			|
			|
		    -
		  	|
		    |
		movement may be interpreted as pitch: up&down
	*/
    LOG_DBG("Sacnning: %d", d);
    // ---- PITCH only ----
    move_savely_by(&steppers[PITCH_IDX],  d);
    move_savely_by(&steppers[PITCH_IDX], -2*d);
    move_savely_by(&steppers[PITCH_IDX],  d);
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
	int32_t d_half = d / 2;

    LOG_DBG("Star pattern, diameter: %d\n", d);

	// check if frequency is the same!
	if (current_frequencies[PITCH_IDX] != current_frequencies[YAW_IDX]){
		LOG_ERR("Star pattern not recommended with different frequencies");
	}

    // ---- PITCH only ----
    move_savely_by(&steppers[PITCH_IDX],  d_half);
    move_savely_by(&steppers[PITCH_IDX], -d);
    move_savely_by(&steppers[PITCH_IDX],  d_half);

    // ---- YAW only ----
    move_savely_by(&steppers[YAW_IDX],  d_half);
    move_savely_by(&steppers[YAW_IDX], -d);
    move_savely_by(&steppers[YAW_IDX],  d_half);

    // ---- BOTH in the same direction ----
    move_savely_by(&steppers[PITCH_IDX],  d_half);
    move_savely_by(&steppers[YAW_IDX],  d_half);
	
	wait_for_movement_to_finish(&steppers[PITCH_IDX]); // wait till movement is finished, to be synchronous
	wait_for_movement_to_finish(&steppers[YAW_IDX]);

    move_savely_by(&steppers[PITCH_IDX],  -d);
    move_savely_by(&steppers[YAW_IDX],  -d);

	wait_for_movement_to_finish(&steppers[PITCH_IDX]); // wait till movement is finished, to be synchronous
	wait_for_movement_to_finish(&steppers[YAW_IDX]);

    move_savely_by(&steppers[PITCH_IDX],  d_half);
    move_savely_by(&steppers[YAW_IDX],  d_half);

    // ---- BOTH in the opposite direction ----
    move_savely_by(&steppers[PITCH_IDX],  d_half);
    move_savely_by(&steppers[YAW_IDX],  -d_half);
	
	wait_for_movement_to_finish(&steppers[PITCH_IDX]); // wait till movement is finished, to be synchronous
	wait_for_movement_to_finish(&steppers[YAW_IDX]);

    move_savely_by(&steppers[PITCH_IDX],  -d);
    move_savely_by(&steppers[YAW_IDX],  d);

	wait_for_movement_to_finish(&steppers[PITCH_IDX]); // wait till movement is finished, to be synchronous
	wait_for_movement_to_finish(&steppers[YAW_IDX]);

    move_savely_by(&steppers[PITCH_IDX],  d_half);
    move_savely_by(&steppers[YAW_IDX],  -d_half);
}

void simple_target(vermin_collector_ros_msgs__msg__Command *cmd){
    LOG_DBG("Simple target");

    move_savely_to(&steppers[PITCH_IDX], cmd->step_goals[PITCH_IDX]);
    move_savely_to(&steppers[YAW_IDX],   cmd->step_goals[YAW_IDX]);
    move_savely_to(&steppers[SLIDE_IDX], cmd->step_goals[SLIDE_IDX]);

	if (cmd->scan_limit > 0){
		LOG_DBG("Starting scan with length %d", cmd->scan_limit);
		scan_pattern(cmd->scan_limit);
	} 
}

void handle_target(vermin_collector_ros_msgs__msg__Command *cmd){
	vermin_collector_ros_msgs__msg__Command local_cmd = *cmd; // copy command locally, sothat new one may arrive
	command_pending = false;
    system_state = STATE_MOVING;
	if (led_strip_update_rgb(strip, &colors[MOVING_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}

    if (local_cmd.star_diameter == 0) {
        simple_target(&local_cmd); // will apply scan if needed
		// activate laser later once movement is finished
    } else {
		local_cmd.scan_limit = 0;
		local_cmd.laser_duration_ms = 0; // reset to avoid further laser activation below
		simple_target(&local_cmd); // move to goal
		// activate laser indefinitely
		activate_led_non_blocking();
        star_pattern(&local_cmd); // execute star
		deactivate_led_non_blocking();
    }

	wait_for_movement_to_finish(&steppers[PITCH_IDX]); // wait till remaining movement is finished, before changing system state
	wait_for_movement_to_finish(&steppers[YAW_IDX]);
	wait_for_movement_to_finish(&steppers[SLIDE_IDX]);
	activate_led_for_ms_blocking(local_cmd.laser_duration_ms);

    system_state = STATE_READY;
	if (led_strip_update_rgb(strip, &colors[READY_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}
}

/* #################################### command thread entry point #################################### */
void command_thread_entry(void *arg_1, void *arg_2, void *arg_3){
	ARG_UNUSED(arg_1);
	ARG_UNUSED(arg_2);
	ARG_UNUSED(arg_3);

	while(1){
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
				handle_homing(simple_homing); // just move to previously stored reference zero
				break;

			case vermin_collector_ros_msgs__msg__Command__SOFT_HOMING:
				handle_homing(soft_homing); // set current position to be the reference, do not move at all
				break;

			case vermin_collector_ros_msgs__msg__Command__HARD_HOMING:
				handle_homing(hard_homing); // perform one revolution on each axis and move to middle of magnets
				break;

			default:
				printk("Unknown command\n");
				break;
			}
    	}
		k_msleep(100);
	}

}




/* #################################### main loop #################################### */
int main(void){
	// define thread tid
	k_tid_t command_tid;

	// init peripherial devices
	init_led();
	init_steppers();
	init_halls();


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



	// start command thread
	command_tid = k_thread_create(&command_thread, 		// thread struct
								  command_stack,		// stack
								  K_THREAD_STACK_SIZEOF(command_stack),
								  command_thread_entry, // Entry point
								  NULL,					// arg_1
								  NULL,					// arg_2
								  NULL,					// arg_3
								  7, 					// Priority
								  0, 					// options
								  K_NO_WAIT				// Delay to creation
	);

	while (1){
		// main thread runs ros execution
		rclc_executor_spin_some(&executor, 100);
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