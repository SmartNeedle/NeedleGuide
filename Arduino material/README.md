# Arduino board and sensors

This subfolder contains all information to install the Arduino board and sensors as well as how to run the ROS node written in "Arduino".

## Hardware Installation

Please follow the circuit diagram provided as a picture in this folder. 

Necessary material:
- Arduino Uno board with a USB cable
- CUI absolute rotary encoder multi-turn 12 bit AMT-22 
- Linear soft potentiometer 200mm from Softpot
- A 3 pin connector to use as an extension of the linear potentiometer
- Male-to-female wires
- Wire AMT-06C-1-036 to connect rotary encoder to Arduino board

## Software Installation 

Please install the Arduino IDE and load the sketch ReadCUIRotaryEncoderAndSoftPotentiometer.ino provided in the Arduino code folder. 

Then, add the rosserial library using the Arduino software menu: Sketch --> Include Library --> Manage Libraries. Look for the Rosserial Arduino Library and install version 0.9.1. Please close and restart the software before going to next step. 

You also need to have ROS installed on your computer (It has been tested with ROS noetic only.)

Please also install the ros1_bridge.

## Usage 

In the Arduino software, select the correct board: Tools--> Board --> Arduino Uno as well as the correct port under Tools --> port. Then, verify (check button) and upload (arrow button) the code to the Arduino board. 

You will need several terminals in order to launch the Arduino ROS node and the bridge.

First, in one terminal source ROS (here using noetic) and start a ROS 1 roscore:
```bash
source /opt/ros/noetic/setup.bash
roscore
```
Second, in another terminal source ROS and launch the ros1 bridge 
```bash
source /opt/ros/noetic/setup.bash
ros2 run ros1_bridge dynamic_bridge
```

Third, in another terminal source ROS and launch the arduino ROS1 node with the correct port (here /dev/ttyACM0)
```bash
source /opt/ros/noetic/setup.bash
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

Finally, in a last terminal you can source the ROS2 environment and launch the main system bringup node. 
For a quick test you can try running only the package that receives the Arduino values:
```bash
. ~/ros2_foxy/ros2-linux/setup.bash
ros2 launch adaptive_guide adaptive_guide_launch.py sim_level:=2
```

You should see the values measured by the sensors published to ROS2 nodes. 

