# Zephyr Setup
The goal of this chapter is to get a working zephyr workspace. At the time of working on this project, the available Micro-Ros version had conflicts with the latest Zephyr version as some file locations changed. Therefore, the Zephyr version 4.2 with the SDK 0.16.9-rc3 were used instead of the latest. This downgrade requires some manual adaptions of the Zephyr source code to get the necessary motor drivers running as expected. All required steps will be explained below. 

## Zephyr Installation
This sections aims to setup a running Zephyr workspace. It is based on instructions described in the Getting Sarted guide at <a href="https://docs.zephyrproject.org/latest/develop/getting_started/index.html#get-zephyr-and-install-python-dependencies">https://docs.zephyrproject. org/latest/develop/getting_started/index.html#get-zephyr-and-install-python-dependencies</a> and the technical note describing the installation of older versions <a href="https://www.zephyrproject.org/managing-multiple-versions-of-the-zephyr-rtos/">https://www.zephyrproject.org/managing-multiple-versions-of-the-zephyr-rtos/</a>. 

As first step, update your OS and install dependencies:
```bash
sudo apt update
sudo apt upgrade

# get dependecies
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget python3-dev python3-venv python3-tk \
  xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

# verify versions, need at least (in order)  3.20.5, 3.12, 1.4.6
cmake --version
python3 --version
dtc --version
```

Then, the respective Zephyr version gets its own workspace and virtual environment (to avoid clashing with ros python dependencies). Further, the tool "west" is installed that handels building and flashing Zephyr applications and both Zephyr and the SDK are installed:
```bash
# prepare zephyr workspace
mkdir ~/zephyr_4_2
python3 -m venv ~/zephyr_4_2/.venv
source ~/zephyr_4_2/.venv/bin/activate

# get west 
pip install west

# get zephyr 4.2.0
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.2.0 zephyr_4_2 
cd zephyr_4_2
west update

# get sdk 0.16.9-rc3
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.19.9-rc3/zephyr-sdk-0.16.9-rc3_linux-x86_64.tar.gz
tar xzvf zephyr-sdk-0.16.9-rc3_linux-x86_64.tar.gz
cd zephyr-sdk-0.16.9-rc3_linux-x86_64
./setup.sh
```

## Activate Environment
Once everything is installed, the follwoing commands should activate your environment allowing you to build and flash Zephyr code onto the ESP32:

```bash
source ~/zephyr_4_2/.venv/bin/activate
source ~/zephyr_4_2/zephyr/zephyr-env.sh
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.9-rc3
```

## Build and Flash Firmware
With an activated and sourced environemnt, building and flashing should be possible via the following commands:

```bash
west build -p always -b esp32s3_devkitc/esp32s3/procpu
west flash
```
Note that the ```-b esps3_devkitc/esp32s3/procpu``` defines the specific board and cpu used on the ESP32 microcontroller. 

## Zephyr Monitor
The Zephyr OS offeres log-capabilities and also print-statements can be received from the microcontroller by using the monitor. It can be activated using the following command. Note that the monitor, similar to the Micro-Ros agent, needs a port and baudrate parameter to setup the communication properly:
```bash
west espressif monitor -b 115200 -p /dev/ttyACM0
```
The specific port needs to be adapted depending on your system but the baudrate is fixed for the project.

## Zephyr Source Code Adaptions
As the stepper-driver source code seems to have changed a little in the latest Zephyr versions compared to the used version 4.2, as for example the "step_width_ns" property was not available originally in version 4.2 of the tmc2209 drivers. 

Therefore, the source code must be adapted a little. In particular, the two files
- step_dir_stepper_common.c
- step_dir_stepper_common.h 

located in this project's repository under ```./code_adaptions/``` must be placed inside ```zephyr_4_2/zephyr/drivers/stepper/step_dir``` and replace the equally named files there. 

Additinally, the ESP32 has no explicit top-value for its counters but uses some kind of alarm functionality instead which is already implemented by the esp32-vendor so that setting a top-value is redirected to implementing an alarm. However, the vendor's counter backbone has one error in it. The set-up routine which is unavoidably called sometime at start-up, already starts the counter using a config-flag which means it produces step signals right at start up which would lead to unintended behaviour. Further, there were some instances, in which the timer did not start again during the execution of a movement. This was probably the case because the ISR status is reset in the last line in the esp32 counter ISR routine after the stepper-driver callback is executed. It seems that it is only a matter of time until the interrupt is triggered again before the previous ISR execution reaches the end and therefore the newly set flag is cleared at the end of the current ISR execution leading to a deadlock from a dead timer and a driver waiting for steps to complete. 

Subsequentially, two changes have to be made in the vendor's code located at ```zephyr_4_2/zephyr/drivers/counter/counter_esp32_tmr.c```. To do so, the euqally named file counter_esp32_tmr.c in the code_adaptions folder must replace the file in the Zephyr workspace (or just move the respective line higher up in the ISR and change the parameter in the init macro as below):

```c
static void counter_esp32_isr(void *arg) {
	...
	// move the following line up as far as possible
	timer_ll_clear_intr_status(data->hal_ctx.dev, TIMER_LL_EVENT_ALARM(data->hal_ctx.timer_id));
    ...
}

// .counter_en was originally TIMER_START
#define ESP32_COUNTER_INIT(idx)                                                                    \
                                                                                                   \
	static struct counter_esp32_data counter_data_##idx;                                       \
                                                                                                   \
	static const struct counter_esp32_config counter_config_##idx = {                          \
		.counter_info = {.max_top_value = UINT32_MAX,                                      \
				 .flags = COUNTER_CONFIG_INFO_COUNT_UP,                            \
				 .channels = 1},                                                   \
		.config =                                                                          \
			{                                                                          \
				.alarm_en = TIMER_ALARM_DIS,                                       \
				.counter_en = TIMER_PAUSE,                                         \
				.intr_type = TIMER_INTR_LEVEL,                                     \
				.counter_dir = TIMER_COUNT_UP,                                     \
				.auto_reload = TIMER_AUTORELOAD_DIS,                               \
				.divider = ESP32_COUNTER_GET_CLK_DIV(idx),                         \
			},                                                                         \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(TIMER(idx))),                            \
		.clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(TIMER(idx), offset),        \
		.group = DT_PROP(TIMER(idx), group),                                               \
		.index = DT_PROP(TIMER(idx), index),                                               \
		.irq_source = DT_IRQ_BY_IDX(TIMER(idx), 0, irq),                                   \
		.irq_priority = DT_IRQ_BY_IDX(TIMER(idx), 0, priority),                            \
		.irq_flags = DT_IRQ_BY_IDX(TIMER(idx), 0, flags)};                                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, counter_esp32_init, NULL, &counter_data_##idx,                  \
			      &counter_config_##idx, PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,   \
			      &counter_api);


```


Finally, there was another tricky bug which is not understood fully but a work-around was found. GPIO38 which is already fixed in the PCB design, later showed strange behaviour in the software as it could not be used properly to enable the pitch stepper driver. It just sticked always to being high which disables the driver. However, adding the following lines to the tmc22xx.c file located in  ```zephyr_4_2/zephyr/drivers/stepper/adi_tmc``` fixes the problem:

```c
static int tmc22xx_stepper_enable(const struct device *dev)
{
	const struct tmc22xx_config *config = dev->config;

	/* FORCE reconfiguration right before use */
    gpio_pin_configure_dt(&config->enable_pin, GPIO_OUTPUT);

	LOG_DBG("Enabling Stepper motor controller %s", dev->name);
	return gpio_pin_set_dt(&config->enable_pin, 1);
}

static int tmc22xx_stepper_disable(const struct device *dev)
{
	const struct tmc22xx_config *config = dev->config;

	/* FORCE reconfiguration right before use */
    gpio_pin_configure_dt(&config->enable_pin, GPIO_OUTPUT);

	LOG_DBG("Disabling Stepper motor controller %s", dev->name);
	return gpio_pin_set_dt(&config->enable_pin, 0);
}
}
```