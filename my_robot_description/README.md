# my_robot_description package

Based on the book _Learning ROS for Robotics Programming_ by Aaron Martinez and Enrique Fernández, chapter 5: _3D Modeling and simulation_.

### URDF

To check whether the sintax is fine or whether it have errors:
```
$ check_urdf my_robot.urdf
```

To get the Graphviz in pdf:
```
$ urdf_to_graphviz my_robot.urdf
```

To run:
```
$ roslaunch my_robot_description my_robot_rviz.launch
```