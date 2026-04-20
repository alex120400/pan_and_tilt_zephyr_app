# Introduction
This documantation serves as starting point for the ESP32-part of the Vermin Collector project. It aims to explain the necessary steps to deploy the developed application code on the ESP32s3-devkitc microcontroller board and the intention behind it. Not every detail of the source code is explained, but the most fundamental parts and their tasks should be covered. Further, important parts such as:

- ROS2 and Microros setup, see [ROS2 and microros setup](./ros.md)
- Zephyr workspace setup, see [Zephyr setup](./zephyr.md)
- topics and messages for communication, see [Topics and messages](./tops_mes.md)
- hardware, like schematic & layout design, motors, sensors, see [Hardware Design](./hardware.md)
- Application code, see [Application Code](./code.md)
- mechanical assembly and positioning accuracy, see [Mechanical Design](./mech.md) 
 

are featured in indivudal chapters. The final assembly is shown in [Figure 1](#assembly).


<div id="assembly" align="center">
  <img src="./images/LaserArm_Assembled.jpeg" alt="Assembled Camera and Laser pointer" width="70%">
  <p><em>Figure 1: Assembled System allowing pan and tilt movement combined with sliding featureing a camera slot and laser pointer.</em></p>
</div>

# References
- https://docs.zephyrproject.org/latest/develop/getting_started/
- https://docs.ros.org/en/humble/
- https://micro.ros.org/docs/tutorials/core/overview/
- https://github.com/isaac879/Pan-Tilt-Mount