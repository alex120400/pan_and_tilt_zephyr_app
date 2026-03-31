# micro-ROS module for Zephyr

This module is based on the module devolpped at https://github.com/micro-ROS/micro_ros_zephyr_module/tree/humble which has been tested in Zephyr RTOS v4.0.0 (SDK 0.16.9-rc3), and v4.1.0 (SDK 0.16.9-rc3), using a docker image based on 'zephyrprojectrtos/zephyr-build:v0.26.17'.



## Purpose of the Project




## Zephyr setup
Get a working version of Zephyr 4.2 and the sdk-0.16.9-rc3 using the following part of the Getting Sarted guide (https://docs.zephyrproject.org/latest/develop/getting_started/index.html#get-zephyr-and-install-python-dependencies) and the technical note describing getting older versions (https://www.zephyrproject.org/managing-multiple-versions-of-the-zephyr-rtos/). 

### Installation steps
Most relevant commands should be:

```bash
sudo apt update
sudo apt upgrade

# get dependecies
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget python3-dev python3-venv python3-tk \
  xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

# verify versions, need in order at least 3.20.5, 3.12, 1.4.6
cmake --version
python3 --version
dtc --version

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
$> tar xzvf zephyr-sdk-0.16.9-rc3_linux-x86_64.tar.gz
$> cd zephyr-sdk-0.16.9-rc3_linux-x86_64
$> ./setup.sh
```

### zephyr source code changes
The ESP32 has no explicit top-value for its counters but uses some kind of alarm functionality instead. However, due to this, the standard counter-functionality implemented in the TMC2209 driver functions a little peculiar. Even though, it should function as the esp32-vendor has adapted its own code so that setting a top-value is redirected to implementing an alarm, the next difference is that esp32-timers do not simply start from zero once the alarm is triggered which may lead to strange behaviour and weird debug-logs on the zephyr monitor. Additionally, the stepper-source code seems to have changed a little in general, as only single spikes were encountered in the v4.2 instead of 50% duty cycles for step signals. Therefore, the source code must be adapted a little. In particular, the three files ./code_adaptions/step_dir_stepper_counter_timing.c, ./code_adaptions/step_dir_stepper_common.c and ./code_adaptions/step_dir_stepper_common.h must be placed inside zephyr_4_2/zephyr/drivers/stepper/step_dir and replace the equally named files there.


### Activate zephyr environment
Once everything is installed, the follwoing commands should activate your environment allowing you to build and flash zephyr code onto the esp32:

```bash
# if not activated, switch to virtual environment
source ~/zephyr_4_2/.venv/bin/activate
# activate zephyr commands and link correct sdk
source ~/zephyr_4_2/zephyr/zephyr-env.sh
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.9-rc3
```

### building zephyr firmware
with a activated and sourced environemnt, building and flashing should be possible via the following commands

```bash
west build -p always -b esp32s3_devkitc/esp32s3/procpu
west flash
```

### monitoring zephyr os
The log messages from the esp32 and zephyr can be monitored with the following commands using a baudrate of 115200 but the port needs to be adapted of course:

```bash
west espressif monitor -b 115200 -p /dev/ttyACM0
```



## Micro ROS Setup

### Dependencies

This component needs `colcon` and other Python 3 packages in order to build micro-ROS packages:

```bash
pip3 install catkin_pkg lark-parser empy colcon-common-extensions
```


### Setup ros-agent
Follow the steps described in the Quickstart, Building and Building micro-ROS-Agent sections from https://github.com/micro-ROS/micro_ros_setup?tab=readme-ov-file#building to get a working ros-agent for the first time. Do not follow any firmware sections!


### Start ros-agent in micro-ros workspace via (if built at least once before)
```bash
source /opt/ros/humble/setup.bash
cd ~/uros_ws
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 460800
```


## ROS2 setup
to interact with the micro-ros agent via ros2, the vermin_collector_ros_msgs package needs to be built there too (ros2 installation required, change "humble" against your respective version):

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone https://phabricator.ict.tuwien.ac.at/source/Vermin_Collector_ROS_Msgs.git

cd ~/ros2_ws
colcon build --packages-select vermin_collector_ros_msgs
source install/setup.bash

# verify it worked via
ros2 interface show vermin_collector_ros_msgs/msg/Command
ros2 interface show vermin_collector_ros_msgs/msg/Feedback

```

Send a command
```bash
ros2 topic pub --once /ESP32_Command vermin_collector_ros_msgs/msg/Command "{
  command_type: 1,
  step_goals: [32000, 0, 0],
  laser_duration_ms: 0,
  star_diameter: 0,
  resolution: 64,
  frequency: 1
}"
```

Listen to feedback
```bash
ros2 topic echo /ESP32_Feedback
```


## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 system_modes,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

There are no known limitations.
