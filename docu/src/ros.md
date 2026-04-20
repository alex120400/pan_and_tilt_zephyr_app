# ROS2 and Microros Setup
The goal of this chapter is to setup a working ROS2 and microros environment ready for communicating with the application. Note that not all steps are stated below as some "getting started" guides from the respective sources are suffcient. Nevertheless, some improtant hints are given here and beyond basic steps are stated.

## ROS2
ROS2 is an open source operating system ment for advanced robot systems. It offeres several communication interfaces for different purposes. However, for the scope of this project, the basic publisher and subscriber tools suffice together with custom messages.
### Installation
To get a working ROS2 workspace, follow the guide https://docs.ros.org/en/humble/Installation.html. After this, you should have a ROS2 workspace located in some folder, in the following referred to as ```~/ros2_ws```, and commands like ```ros2 topic list``` should work once its activated. Always activate the workspace by moving to ```~/ros2_ws``` and call ```source /opt/ros/humble/setup.bash``` (on Ubuntu at least). Note that during the project, the ROS2 distribution "humble" was used. Therefore, in some commands you might need to change the word "humble" for your respective disrtibution. 

### Custom Messages
The application is built on custom messages (Feedback and Command) that are used in a publisher and subscriber fashion. To get these messages, clone the respective git repo into your src folder in your ROS2 workspace:

```bash
source /opt/ros/humble/setup.bash # activate ROS2
cd ~/ros2_ws/src # move to src folder
git clone https://phabricator.ict.tuwien.ac.at/source/Vermin_Collector_ROS_Msgs.git
```

Once you have cloned the repository, you need to build the messages to be actually usable:
```bash
cd ~/ros2_ws # go back to ws folder
colcon build --packages-select vermin_collector_ros_msgs
```
Once this is done successfully, you need to source the local setup bash-file. Otherwise you cannot use the newly built messages:
```bash
source install/setup.bash

# verify it worked via
ros2 interface show vermin_collector_ros_msgs/msg/Command
ros2 interface show vermin_collector_ros_msgs/msg/Feedback
```

## Microros
Microros builds a bridge for ros-communication connecting powerfull computers to constrained microcontroller platforms. Again, we need just some minimal setup and do not explore its full power. The following subsections are based on instructions found in the sections "Quickstart" and "Building micro-ROS-agent" in the following guide https://github.com/micro-ROS/micro_ros_setup. Avoid any firmware steps from that guide as this project uses the standalone microros version for integration into zephyr!

### Installation
You will need a working ROS2 workspace, colcon and some other python dependencies:
```bash
pip3 install catkin_pkg lark-parser empy colcon-common-extensions
```
After that, we need to clone the microros setup repository and build the package in a separate microros workspace, reffered to in the following as ```uros_ws```:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash # insert your ros distro and activate workspace

mkdir uros_ws && cd uros_ws # create microros workspace
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build
source install/local_setup.bash
```

### Building the Agent
Once the microros installation is complete, you can create an agent that actually manages communication between a ros master (possibly a computer) and your microcontroller. To do so, run the following commands to build the agent:
```bash
# if not done before already, activate ros2 workspace and move to micro ros workspace
source /opt/ros/humble/setup.bash
cd ~/uros_ws
source install/local_setup.sh

# build the agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```

### Running the agent
Once the agent has been built, it can be simply run each time it is needed. An agent needs a port and a baudrate to know on where it should contact your microcontroller and with what speed they communicate. Therefore always run the agent with the following parameters but note that the port depends on your setup and the baudrate is specific to this project:
```bash
# if not done before already, activate ros2 workspace and move to micro ros workspace
source /opt/ros/humble/setup.bash
cd ~/uros_ws
source install/local_setup.sh

# run the agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -b 460800
```

Note that the zephyr monitor (explained in [Zephyr setup](./zephyr.md)) resets the ESP32 when started and at least when working with WSL this might disconnect the ros-port from ubuntu. Therefore, always start the monitor first and then activate the ros agent once your prot is visible in the WSL. Helpful commands to connect physical ports to your WSL distribution are the following:

```bash
usbipd list # checkout which ports are available 

usbipd attach --wsl --busid x-y # insert actual id, might need to share port prior to attach
```

Note that sometime in the near past, the WSL ubuntu internals were changed meaning it was modularized. This author is no linux expert, but it seems that this caused some drivers to be missing by default and communication with classic serial devices on phyiscal ports has become more difficult. Look into  https://learn.microsoft.com/en-us/windows/wsl/connect-usb and if it works already - nice! If not, there must be extra work done. As help, checkout the following discussion https://askubuntu.com/questions/1551098/wsl-kernel-module-ch341-not-loaded-loading-automatically?noredirect=1&lq=1.

