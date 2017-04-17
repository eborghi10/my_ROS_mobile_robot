# Differential drive mobile robot using ROS

1) **AS5048-ros-node**: rosserial package for read data from two AS5048's magnetic encoder using Arduino MEGA2560, and publish to "_/encoder/left_" and "_/encoder/right_" topics.

2) **from_keyboard_node**: node that reads a _Twist_ message from the topic "_/cmd_vel_mux/input/teleop_" and moves the DC motors of the differential drive robot.

3) **from_keyboard_node_test**: lightweight implementation of the code _from_keyboard_node_. It uses local variables inside the class instead of global variables.

4) **pid_wheels**: example of an action client & server. Will be hosted in the Raspberry.

5) **arduino_actuators**: integration of the first two packages (_AS5048-ros-node_ and _from_keyboard_node_) into a single package.

6) **arduino_core**: using _arduino_actuators_ as base package, implements a "_bridge_" between _rosserial_ and the actions package _pid_wheels_.