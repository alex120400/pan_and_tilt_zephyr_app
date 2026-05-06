# Mechanical Design
This chapter introduces some details to the mechanical design. It is mostly based on this youtube series <a href="https://www.youtube.com/watch?v=uJO7mv4-0PY">https://www.youtube.com/watch?v=uJO7mv4-0PY </a>.

## Tilt Mount (Pitch motor)
The tilt mount is completey reworked to firmly house the ZED2i stero camera inside and a powerful (multiple Watts) Laser ontop. It also offers 3 differnt holes for mouning to allow for a balanced center of mass with different setups. A 3D rendering can be seen below in [Figure 11](#tilt-mount).

<div id="tilt-mount" align="center">
  <img src="./images/tilt_mount.png" alt="Tilt mount rendering" width="70%">
  <p><em>Figure 11: 3D rendering of the tilt mount holding the camera. </em></p>
</div>

## Roll Mount
The Roll mount is widened to allow the stereo camera to fit, as shown below in [Figure 12](#roll-mount).

<div id="roll-mount" align="center">
  <img src="./images/pan_mount.png" alt="Roll mount rendering" width="70%">
  <p><em>Figure 12: 3D rendering of the roll mount, which is the base holding both the PCB and the full tilt mount. </em></p>
</div>

## Laser Housing
A enclosure is designed to give a small 1mW "Arduino Laser" the same dimensions as the powerful lasers used in the final assembly. This allows for safe testing. A rending of the design is shown below in [Figure 13](#laser-mount).

<div id="laser-housing" align="center">
  <img src="./images/laser_housing.png" alt="Laser Housing" width="70%">
  <p><em>Figure 13: 3D rendering of the laser housing, suited for both a 1mW Arduino Laser and a more powerful laser for the final application of vermin termination. </em></p>
</div>


## Full assembly 
The full assembly is shown below in [Figure 14](#proto-assembly). The only things missing are the slide mount which is still in design as well as the camera and the laser which have not yet been mounted. The ESP32 is missing the usb cables necessary for communication.

<div id="proto-assembly" align="center">
  <img src="./images/proto_assembly.jpeg" alt="Prototype assembly with the roll and tilt motors as well as the pcb" width="70%">
  <p><em>Figure 14: The prototype assembly with the mounted PCB and both the roll and tilt motors. The usb cables, the camera and the laser mount are missing and the slide mount is not shown either. </em></p>
</div>

## Hall Sensor Tips
The used a3144 digital hall sensors do not have a strong sensitivity but there is a preferred direction of the magnitc field. Be awere of that before glueing any of the magnets and test it beforehand with the hall sensors in their final configuration. Further, the tilt (pitch motor) hall sensor mount is not suitable as planned by the reference project under <a href="https://github.com/isaac879/Pan-Tilt-Mount"> https://github.com/isaac879/Pan-Tilt-Mount </a> because the sensor is not sensitive enough to recognize the magnetic field through about 1 cm of plastic. Therefore the sensor had to be put manually closer to the magnet mount to get a sufficently strong signal.

## Positioning Accuracy
In this section, the achievable accuracies for all mounts is given for different microstep resolutions, see [Figure 15](#pitch-yaw-accuracy) and [Figure 16](#slide-accuracy). Note again that the resolution for all steppers is synchronous due to limited GPIOs available on the ESP32. Further note that these are calculations and may be limited by mechanical inaccuracies.


<div id="pitch-yaw-accuracy" align="center">
  <img src="./images/pitch_yaw_accuracy.jpg" alt="Angualar accuracy calculations of the pitch and yaw mounts" width="70%">
  <p><em>Figure 15: Proposed angular accuracy of the tilt (=pitch) and roll (=yaw) mounts for different microstep resolutions. </em></p>
</div>

<div id="slide-accuracy" align="center">
  <img src="./images/slide_accuracy.jpg" alt="Lateral accuracy calculations of the slide mount" width="70%">
  <p><em>Figure 16: Proposed lateral accuracy of the slide mount for different microstep resolutions. </em></p>
</div>