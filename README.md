# Differential drive mobile robot using ROS

1) **AS5048-ros-node**: rosserial package for read data from two AS5048's magnetic encoder using Arduino MEGA2560, and publish to "_/encoder/left_" and "_/encoder/right_" topics.

2) **from_keyboard_node**: node that reads a _Twist_ message from the topic "_/cmd_vel_mux/input/teleop_" and moves the DC motors of the differential drive robot.

3) **pid_wheels**: example of an action client & server.

4) **arduino_actuators**: integration of the first two packages (_AS5048-ros-node_ and _from_keyboard_node_) into a single package.

5) **pid_ddmr_wheels_server**: action server for the PID control of the DC motor of the differential drive mobile robot.

6) **arduino_core**: merging the _arduino_actuators_ code and the action client of the _pid_wheels_ package.