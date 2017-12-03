# my_robot_base

## Debugging

Install: 

```
sudo apt-get install ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-rqt-controller-manager
```

Execute: `rosrun rqt_controller_manager rqt_controller_manager`

## Executing

`roslaunch my_robot_base hw_control.launch`

This file will execute a node named `my_robot_base_node` with the following parameters:

```
**Publications: **
 * /my_robot/left_wheel_vel [std_msgs/Float32]
 * /rosout [rosgraph_msgs/Log]
 * /mobile_base_controller/odom [nav_msgs/Odometry]
 * /tf [tf/tfMessage]
 * /my_robot/right_wheel_vel [std_msgs/Float32]

**Subscriptions: ** 
 * /my_robot/right_wheel_angle [unknown type]
 * /mobile_base_controller/cmd_vel [unknown type]
 * /my_robot/left_wheel_angle [unknown type]

**Services: **
 * /controller_manager/list_controller_types
 * /controller_manager/reload_controller_libraries
 * /controller_manager/switch_controller
 * /start
 * /my_robot_base_node/get_loggers
 * /controller_manager/load_controller
 * /my_robot_base_node/set_logger_level
 * /controller_manager/unload_controller
 * /controller_manager/list_controllers
 * /stop
```

Besides, you can publish sensor data with Arduino and receive motor velocities using:
`roslaunch arduino_peripherals run_arduino_node.launch`