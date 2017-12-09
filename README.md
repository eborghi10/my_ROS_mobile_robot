# Differential-drive mobile robot using ROS

To see the robot in rviz, read the _README_ file inside the `my_robot_description` package.

To see the complete robot simulation (using rviz and Gazebo), read the _README_ file inside the `my_robot_gazebo` package.

To execute `ros_control` in the real robot, read the _README_ file inside the `my_robot_base` package.

To see the implementation of `slam_gmapping` for _Simulatenous localization and mapping_, read the _README_ file inside the `my_robot_navigation` package.

----

Nonsense tip to record the screen: `ffmpeg -f x11grab -s 1366x768 -r 15 -i :0.0 -s 1366x768 -r 15 -qscale 0 video.avi`

## Install

```
$ sudo apt-get install ros-kinetic-depthimage-to-laserscan
$ sudo apt-get install ros-kinetic-joint-state-controller
$ sudo apt-get install ros-kinetic-effort-controllers
$ sudo apt-get install ros-kinetic-position-controllers
$ sudo apt-get install ros-kinetic-controller-manager
```
