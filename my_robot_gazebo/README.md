# My Robot Gazebo

Gazebo files to test the robot in the simulator.

### Build

Build package: `catkin_make --only_package_deps my_robot_gazebo` and `source devel/setup.bash`.

Install dependencies: `sudo apt-get install ros-kinetic-ros-control` and `sudo apt-get install ros-kinetic-controller-manager`.

### Test

Test the robot xacro:
```
$ roslaunch my_robot_gazebo my_robot_world.launch paused:=true
```

Launch Gazebo & rviz simulations:
```
$ roslaunch my_robot_gazebo my_robot_world.launch
```

Move with the keyboard:
```
$ rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/my_robot/cmd_vel
```

![Gazebo simulation](../resources/gazebo.jpg)
