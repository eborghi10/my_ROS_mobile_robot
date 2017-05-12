# my_robot_description package

### xacro

To convert the xacro file into a URDF file:
```
$ rosrun xacro xacro my_robot.xacro > my_robot.urdf
```

### URDF

To check whether the sintax is fine or whether it have errors:
```
$ check_urdf my_robot.urdf
```

To get the Graphviz in pdf:
```
$ urdf_to_graphviz my_robot.urdf
```

### Test

To run rviz test:
```
$ roslaunch my_robot_description my_robot_rviz.launch
```

![Chassis](../resources/robot_chassis.jpg)

![GraphViz](../resources/graphviz.png)