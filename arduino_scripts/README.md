# Differential drive mobile robot using ROS

To launch the robot use: `$ roslaunch pid_wheels execute.launch`

1) **AS5048-ros-node**: rosserial package for read data from two AS5048's magnetic encoder using Arduino MEGA2560, and publish to "_/encoder/left_" and "_/encoder/right_" topics.

2) **from_keyboard_node**: node that reads a _Twist_ message from the topic "_/cmd_vel_mux/input/teleop_" and moves the DC motors of the differential drive robot.

3) **from_keyboard_node_test**: lightweight implementation of the code _from_keyboard_node_. It uses local variables inside the class instead of global variables.

4) **pid_wheels_action_server**: the action server do the math for the PID. The action client interacts with the PID action server and with the _control_node_. This package will be hosted in the Raspberry Pi.

5) **arduino_actuators**: integration of the first two packages (_AS5048-ros-node_ and _from_keyboard_node_) into a single package.

6) **robot_msgs**: custom messages for motors and encoders. From here, I'll implement a new approach.

7) **control_node**: receives the keyboard commands in order to translate into wheel velocities, then, sends them to the action client.

8) **arduino_actuators_new**: similar to _arduino_actuator_ but it uses sensor_msgs::JointState for the encoder.

9) **my_robot_simulator**: all the packages that involve the simulation in Gazebo.

----

## New approach

![NewApproach](resources/software.png)

The _turtlebot_teleop_ package is used to send, from the keyboard, the linear and angular velocity needed (_geometry_msgs::Twist_).

This data is sent via _/cmd_vel_mux/input/teleop_ topic and received by the **control node**. This node, transforms the data into wheel velocities (left and right from a differential drive mobile robot).

This modificated information is given to the **action client** who operates with the **action server** in order to perform the PID calculations.

The closed loop system needs the encoder data provided by the **Arduino rosserial node** via the _/encoder_ topic in order to work fine.

Besides, the action server sends the current velocity for each loop, also known as PWM values for the DC motors.

All the information managed by the Arduino is a custom message package (_robot_msgs::Arduino_) which consists of two parameters: a string (left or right, wheel or encoder indistinctly), and a float32 data value.

![RqtGraph](resources/rqt_graph.png)