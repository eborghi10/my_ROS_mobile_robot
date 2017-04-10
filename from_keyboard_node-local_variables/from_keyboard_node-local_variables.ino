/**
 * NOTES:
 * 
 * To use the keyboard:
 * 
 * $ roslaunch turtlebot_teleop keyboard_teleop.launch
 * 
 * This node publishes into "/cmd_vel_mux/input/teleop" which has the following
 * format (geometry_msgs/Twist):
 * 
 * geometry_msgs/Vector3 linear: float {x,y,z}
 * geometry_msgs/Vector3 angular: float {x,y,z}
 *       
 * To execute the program correctly:
 * 
 * $ roslaunch turtlebot_teleop keyboard_teleop.launch
 * $ rosrun rosserial_python serial_node.py /dev/ttyACM0
 *
 * http://wiki.ros.org/rosserial_python
 *
 * TODO:
 * - catch CTRL-C and exit cleanly (stopping the robot)
 * - how to obtain the maximum velocity (modify bounds)
 * 
 */

#include "DifferentialDriveRobot.h"

DifferentialDriveRobot *my_robot;

void setup()
{
  my_robot = new DifferentialDriveRobot(
    new DCMotor(IN1, IN2),
    new DCMotor(IN3, IN4));

  my_robot->updateParameters(0.2, 1.0);
}

void loop() 
{  
    (my_robot->nh).spinOnce();
    delay(1);   
}