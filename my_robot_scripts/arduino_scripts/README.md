# Differential drive mobile robot using ROS
===========================================

These are simple Arduino scripts to test the mobile robot hardware.

1) **AS5048-ros-node**: rosserial package for read data from two AS5048's magnetic encoder using Arduino MEGA2560, and publish to "_/encoder/left_" and "_/encoder/right_" topics.

2) **from_keyboard_node**: node that reads a _Twist_ message from the topic "_/cmd_vel_mux/input/teleop_" and moves the DC motors of the differential drive robot.

3) **from_keyboard_node_test**: lightweight implementation of the code _from_keyboard_node_. It uses local variables inside the class instead of global variables.

---

**Important package:** `arduino_peripherals`

Used by `ros_control` as `hardware_interface`.

- Upload the code into the Arduino
- Execute the node using:

```bash
$ roslaunch arduino_peripherals run_arduino_node.launch
```
