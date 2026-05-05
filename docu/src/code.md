# Application Code
This chapter aims to explain the most relevant components of a zephyr project and how to connect it to microros. Further, it will explain the most relevant parts of the application firmware that runs on the ESP32 and what the desired functionality is. It does not provide a full picture of the source code but rather states the ideas behind it.

## Zephyr project structure
The application itself builds on the separated micro-ros module struture for zephyr development found [here](https://github.com/micro-ROS/micro_ros_zephyr_module). It features the correct setup of microros functionality within a zephyr application but allows to use zephyr's rich building tool west to build and flash your applications on the microcontroller. 

Each zephyr application needs the following files:
- src/main.c
- app.overlay (can be named differntly if explicitly specified in the build command, this name is looked for by default) 
- prj.conf 

These three files activate and control hardware on your microcontroller. The prj.conf file defines relevant parameters like stack-sizes but also loads and activates relevant hardware libraries and functionality. For example ```CONFIG_COUNTER=y``` is necessary to drive the stepper motors with actual hardware timers and not software timers managed by the operating system. For further flags and paramters, look at the [source file](../../prj.conf). The app.overlay file connects certain functionality like counters and stepper drivers together and offers access to these in the main.c file. For example, the setup of the pitch stepper driver and its counter is shown below:
```
&timer0 {
    status = "okay";
	/* pitch counter */
    counter0: counter {
        status = "okay";
    };
};

# the / identifier marks the root node
/ {
    pitch: tmc2209_pitch {
		compatible = "adi,tmc2209";
		en-gpios = <&gpio1 6 GPIO_ACTIVE_LOW>; // gpio1 controller owns pins >31 -> 38 - 32 = 6
		step-gpios = <&gpio0 21 GPIO_ACTIVE_HIGH>;
		dir-gpios = <&gpio1 15 GPIO_ACTIVE_HIGH>; // 47 - 32 = 15
		msx-gpios = <&gpio0 9 GPIO_ACTIVE_HIGH>,
                  <&gpio0 10 GPIO_ACTIVE_HIGH>;
		//dual-edge-step;
		micro-step-res = <16>;
		counter = <&counter0>;
	};
}
```

This setup-snippet shows how to connect GPIOs to the driver so that functions like "stepper_enable" do the job of pulling GPIO38 low and activate the pitch stepper driver. For further information, look at the [source file](../../app.overlay).

Finally, the src/main.c file holds the typical application c-code that is executed with the standard main function and a while(1) loop. Through defined "aliases" in the app.overlay file, certain "devices" can be accessed in the main file:

```
# in the overlay file the alias is defined, note that "-" get translated to "_" in the c file and "_" seem to be not useable in the overlay file
/ {
    aliases {
		pitch-stepper = &pitch; # here comes the name/label of the driver as defined above
	};
}
```

```c
// in the main file the alias is necessary to find the device
const struct device *pitch_dev = DEVICE_DT_GET(DT_ALIAS(pitch_stepper));
```


## Microros communication in zephyr
As the microros agent needs a way of communicating with the microcontroller, certain flags need to be set in the prj.conf like the "CONFIG_MICROROS_TRANSPORT_SERIAL=y" which enables serial communication. In the app.overlay file, the "usb_serial" peripheral needs to be activated for the communication to work:

```
&usb_serial {
	status = "okay";
	current-speed = <460800>; /* baudrate for micro-ros agent */
};
```

As this is a special serial node that has its own physical usb-port on the ESP32S3 devkitC, we need to change the node name in the microros library file "micro_ros_zephyr_module/modules/libmicroros/microros_transports/ serial/microros_transports.c":

```
// original: #define UART_NODE DT_NODELABEL(usart1)
#define UART_NODE DT_NODELABEL(usb_serial)
```

## main.c file structure
This subsection goes over relevant parts of the main file and aims to explain them.

### Peripherals
Besides the stepper drivers, there is a RGB led and three hall sensors that are being used as external peripherals. Each of those peripherals need to be initialized and verified to be ready prefearably before the while loop begins. The RGB Led is used to give visual feedback on what the ESP32 is doing and represents its state. The hall-sensors have pull-up resitors but pull the level Low when near the magnet. The GPIOs connected to the signal line of the hall sensors act as external interupt lines and are used in the hard homing sequence. 

### Threads
The application runs two threads that interact via one another by setting the system state and a command-pending-flag. The "main"-thread contains the while-loop of the main function. It runs the microros-functionality and manages the subscription to the \EPS32_Command topic as well as the publication of the feedback to the \ESP32_Feedback topic. Some relevant parts of the source code are listed below:

```c

int main(void){
	... // initialize peripherals, set system state
	
	rcl_ret_t ret;
	do {  // create init_options
    	ret = rclc_support_init(&support, 0, NULL, &allocator); // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); -> fails with ubuntu wsl
    	if (ret != RCL_RET_OK) {
        	printk("Waiting for micro-ROS agent...\n"); // workaround since reset disconnects serial port from ubuntu wsl, wait till its back
        	k_sleep(K_MSEC(500));
    	}
	} while (ret != RCL_RET_OK);

	rcl_node_t node; // create node
	RCCHECK(rclc_node_init_default(&node, "zephyr_esp32_stepper_manager", "", &support));

	RCCHECK(rclc_publisher_init_default( // create publisher
		&feedback_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(vermin_collector_ros_msgs, msg, Feedback),
		"ESP32_Feedback"));

	rcl_timer_t timer; // create timer for feedback publication
	const uint32_t timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	RCCHECK(rclc_subscription_init_default( // create subscriber
		&command_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(vermin_collector_ros_msgs, msg, Command),
		"ESP32_Command"));

	rclc_executor_t executor; 	// create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 2 is number of handles = subscriptions + timers
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&command_sub,
		&active_command_msg,
		command_callback,
		ON_NEW_DATA));

	... // set system state to ready and start command-thread

	while (1){ // main thread runs ros execution
	rclc_executor_spin_some(&executor, 100);
	k_msleep(100);
	}

	RCCHECK(rclc_executor_fini(&executor)) // free resources
	RCCHECK(rcl_publisher_fini(&feedback_pub, &node))
	RCCHECK(rcl_subscription_fini(&command_sub, &node))
	RCCHECK(rcl_timer_fini(&timer))
	RCCHECK(rcl_node_fini(&node))
	RCCHECK(rclc_support_fini(&support))

	return 0;
}
```
The feedback-publication has a period of 1 second so far and publishes the current step positions, as well as the system state and stepper configuration, the exact structure can be reviewed in [Topics and messages](./tops_mes.md)

The "command_callback" is called as soon as a command is received on the \ESP32_Command topic but note that the system must be in the "READY" state, otherwise the command is ignored. Further, it sets the command-pending-flag that is relevant in the second thread, the command-thread:
```c
void command_callback(const void *msgin){	// handles subscription to new incoming command
	const vermin_collector_ros_msgs__msg__Command *msg = msgin;
	LOG_INF("Received Command of type: %d\n0..SETUP, 1..TARGET, 2..HOMING, 3..SOFT_HOMING, 4..HARD_HOMING\n", msg->command_type);

	if (system_state == STATE_READY){
		command_pending = true; // signal for command thread to act
	} else {
		LOG_INF("Ignored Command of type: %d\nSystem state is %d\n1..CONFIGURING, 2..MOVING, 3..CALIBRATING -> wait for 0..READY\n", msg->command_type);
	}	
}
```

The command-thread is started right before the main-while loop is entered and it runs the following function that holds its own while(1)-loop:
```c
void command_thread_entry(void *arg_1, void *arg_2, void *arg_3){
	ARG_UNUSED(arg_1); // could be used, but not relevant here as important variables are global
	ARG_UNUSED(arg_2);
	ARG_UNUSED(arg_3);

	while(1){
		if (command_pending) {
			// command_pending = false; // done in routines
			switch (active_command_msg.command_type){
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
```
Within the handle-xy functions, the fitting system-state, as described below, is set and the respective function acts accordingly.

### System states
The ESP32 can be in the following states that match the Feedback-message state types:
- READY -> may receive new commands, Led is white
- CONFIGURING -> received a command of type "SETUP", Led is red, but this is usally very fast so it might not be visible
- MOVING -> received a command of type "TARGET", led is green and steppers are moving
- CALIBRATING -> received one of the homing commands, led is blue, can be short or longer in case of "HARD_HOMING"

In the ready state, ESP32 should receive new commands and act accordingly. Use the Feedback state to wait for this case and send commands only then. Otherwise, they are ignored.

#### MOVING State
This state is reached when a command with the type "TARGET" was received. The respective handler called within the command-thread then executes the following functionality:
```c
void handle_target(vermin_collector_ros_msgs__msg__Command *cmd){
	vermin_collector_ros_msgs__msg__Command local_cmd = *cmd; // copy command locally, sothat new one may arrive
	command_pending = false; // reset the flag
    system_state = STATE_MOVING; // set fitting system state
	if (led_strip_update_rgb(strip, &colors[MOVING_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}

    if (local_cmd.star_diameter == 0) {
        simple_target(&local_cmd);
		// activate laser?
    } else {
		local_cmd.scan_limit = 0;
		simple_target(&local_cmd); // move to goal
		// activate laser?
        star_pattern(&local_cmd); // execute star
    }

	wait_for_movement_to_finish(&steppers[PITCH_IDX]); // wait till remaining movement is finished, before changing system state
	wait_for_movement_to_finish(&steppers[YAW_IDX]);
	wait_for_movement_to_finish(&steppers[SLIDE_IDX]);

    system_state = STATE_READY;
	if (led_strip_update_rgb(strip, &colors[READY_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}
}
```

In any case, the stepper drives to a simple position given by the step goals. Then, depending on the star_diameter value, it either executes a star pattern, already described in [Topics and messages](./tops_mes.md) and visualized in [Figure 3](#patterns), or if the scan_limit is greater than zero, a scanning movement also visualized in [Figure 3](#patterns). Note that the star-pattern takes precedence over the scan-limit if provided (= having a value larger than zero).

From the Ros-Master perspective, the followng scenearios should trigger the respecive chain of execution:
- getting data = scanning: Robot may be moving or not. Calculate a fixed array of (absolute) positions that should be visited along the path where data is accumulated continiously.Send position goals one by one to the ESP32 and provide a scan-limit to get more data from each position. Always wait for "READY" state before sending the next position. Robot speed needs to match the execution speed properly.
- Eliminating eggs/vermin: depending on the goal-size, a simple-target may suffice or a star-pattern needs to be executed. In the simple target, the laser will be activated for the provided time given by the "laser_duration_ms" parameter. In the star-pattern, the laser is activated throughout the execution.

#### CONFIGURING State
This state is reached as soon as a "SETUP"-type command arrives and takes only a short time. The LED getting red might not be visible. Through the command parameter "resolution" the stepper resolution can be changed for all steppers synchronously. The default is 16. Through the array-like paramters "frequency_goals" and "en_motors" the stepping frequency for each stepper may be tuned individually as well as if the motor should be on or off:
```c
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
```
From the perspective of the ros master, the following guidelines apply:
- change to a higher frequency depending on the preferred accuracy and speed of the respective movement. E.g. the yaw gear transmission ratio is rather large, which means reaching a position goal may take a while with a smaller frequency. 
- If a motor gets stuck or needs to be moved manually, disable the respective motor through a setup command and a 0 at the respective position in the en_motors array. After that, the motor can be moved freely.
- Use the resolution provided in the Feedback or store it prior to sending a setup-change to calculate the correct absolute step goals accordingly. The full-step amounts per revolution are stated in [Mechanical Design](./mech.md) and need to be scaled using the applied resolution.


#### CALIBRATING State
This state is reached via three slightly different command-types. Namely the "HOMING"-, the "SOFT_HOMING"- and the "HARD_HOMING"-type. All of these have a similar agenda but reach it differently. Right after activation, the ESP32 marks the current motor positions as 0. SO if a movement is executed and then a "HOMING" command arrives, it will move the motors back to their previously set reference. The reference can be changed via "SOFT_HOMING" and "HARD_HOMING":

- "SOFT_HOMING" should be used if the motor was ALREADY moved to a specific position that shall be the new reference. So if for example the ros-master uses a certain camera-driven homing logic, it should provide the ESP32 with step goals to reach that position and then send a "SOFT_HOMING" command to mark this position as reference. 
- "HARD_HOMING" does not need an external reference. It uses magnets (crrently only available for pitch and yaw movement!) and hall sensors to find a certain hard-manufactured home position. The used hall sensors, see [Hardware Design](../book/hardware.html) drive the signal line low when in the range of a magnetic field. Therefore the motors move one revolution and store ESP32 saves the step-positions where the magnetic-field was entered and where it was left. After one full revolution, it moved to motor to the middle between these two positions. 

Note that after the execution of both "SOFT_HOMING" and "HARD_HOMING" a new reference position has been set and that the current steps (e.g. received in the feedback-message) are reset to zero.

The respective handler is given below:

````c
void handle_homing(homing_flag_t homing_flag){
    system_state = STATE_CALIBRATING;
	command_pending = false;
	if (led_strip_update_rgb(strip, &colors[CALIBRATING_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}

	uint32_t home_goal = 0x00;

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
	} else { // homing, just move to last set 0 position
		move_savely_to(&steppers[PITCH_IDX], 0);
		move_savely_to(&steppers[YAW_IDX], 0);
		wait_for_movement_to_finish(&steppers[PITCH_IDX]);
		wait_for_movement_to_finish(&steppers[YAW_IDX]);
	}
	
    system_state = STATE_READY;
	if (led_strip_update_rgb(strip, &colors[READY_IDX], STRIP_NUM_PIXELS) != 0) {
    	LOG_ERR("LED update failed");
	}
}
```



