# micro-ROS Motor Control (ESP32)

This repository contains ESP32 firmware for motor control using micro-ROS.
The ESP32 subscribes to `/cmd_vel` messages from ROS 2 and converts them
into PWM signals for driving a differential-drive mobile robot.

## Features
- micro-ROS over USB serial
- Subscribes to `/cmd_vel` (geometry_msgs/Twist)
- Differential drive motor control
- Safety stop on command timeout

## Usage
1. Flash the ESP32 using Arduino IDE
2. Connect ESP32 via USB
3. Run micro-ROS agent on the PC:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

4. Send velocity commands from Nav2 or terminal

## Related Project

This firmware is part of a larger GPS-based autonomous navigation system
implemented using ROS 2, Nav2, SLAM, and LiDAR.

Main navigation repository:
https://github.com/KavyaSivakumar2006/gps-based-autonomous-navigation
