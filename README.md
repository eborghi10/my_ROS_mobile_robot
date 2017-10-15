# Differential drive mobile robot using ROS

To see the robot in rviz, read the _README_ file inside the `my_robot_description` package.

To see the complete robot (in rviz and Gazebo), read the _README_ file inside the `my_robot_gazebo` package.

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

# ToDo

Adding sensors:

https://bharat-robotics.github.io/blog/adding-hokuyo-laser-to-turtlebot-in-gazebo-for-simulation/

http://wiki.ros.org/amcl

https://www.packtpub.com/mapt/book/hardware_and_creative/9781783554713/9/ch09lvl1sec80/gmapping-and-localization-in-chefbot
