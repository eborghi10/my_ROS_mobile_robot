# My Robot Base
===============

## Build

```bash
$ catkin_make --only_pkg_with_deps my_robot_base arduino_peripherals
$ source devel/setup.bash
```

## Install 

```
sudo apt-get install ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-rqt-controller-manager
```

## Test

To run the `ros_control` node, use:

```bash
$ roslaunch my_robot_base hw_control.launch
```

Besides, you can publish sensor data with Arduino and receive motor velocities using:

```bash
$ roslaunch arduino_peripherals run_arduino_node.launch
```

And to send velocity commands to the robot:

```bash
$ rostopic pub -r 100 /mobile_base_controller/cmd_vel geometry_msgs/Twist TAB-TAB
```
