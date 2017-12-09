# My Robot Navigation
=====================

Navigation stack for implementing `slam_gmapping`.

### Build

```bash
$ catkin_make --only_pkg_with_deps my_robot_navigation
$ source devel/setup.bash
```

### Test

#### Simultaneous Localization and Mapping (SLAM)

```bash
$ roslaunch my_robot_navigation slam_gmapping.launch
```

#### Navigation

```bash
$ roslaunch my_robot_navigation navigate.launch
$ rosrun my_robot_navigation send_goal <x y w>
```