/**
 * http://answers.ros.org/question/232045/rosserial-multiple-publisher-and-subscriber/
 *
 */

#include "include/DifferentialDriveRobot.h"

//////////////////////////////////////////////////////////////////

DifferentialDriveRobot *my_robot;

//////////////////////////////////////////////////////////////////

void setup()
{
  my_robot = new DifferentialDriveRobot(
    new DCMotor(4,5, new MagneticEncoder(10, LEFT, ZERO_TO_2PI)),
    new DCMotor(6,7, new MagneticEncoder(11, RIGHT, ZERO_TO_2PI))
  );

  my_robot->UpdatePhysicalParameters(0.2, 1.0);
}

//////////////////////////////////////////////////////////////////

void loop() 
{
  (my_robot->nh).spinOnce();
  
  // TODO: PUT THE FOLLOWING INTO A FUNCTION

  msg_angle.header.stamp  = my_robot->nh.now();

  dT = msg_angle.header.stamp.toSec() - prevTime.toSec();

  msg_angle.name[LEFT]      = my_robot->GetEncoderTopicName(LEFT);
  msg_angle.position[LEFT]  = my_robot->GetEncoderAngle(LEFT);
  msg_angle.velocity[LEFT]  
    = (msg_angle.position[LEFT] - prevPosition[LEFT]) / dT;

  msg_angle.name[RIGHT]      = my_robot->GetEncoderTopicName(RIGHT);
  msg_angle.position[RIGHT]  = my_robot->GetEncoderAngle(RIGHT);
  msg_angle.velocity[RIGHT]  
    = (msg_angle.position[RIGHT] - prevPosition[RIGHT]) / dT;

  prevPosition[LEFT]  = msg_angle.position[LEFT];
  prevPosition[RIGHT] = msg_angle.position[RIGHT];

  prevTime = msg_angle.header.stamp;

  pub.publish(&msg_angle);

  if( !(my_robot->nh).connected() )  my_robot->Stop();
}

//////////////////////////////////////////////////////////////////

// TODO: IT'S NECESSARY TO DECLARE THE CALLBACK IN THE MAIN LOOP?
//		 TRY TO PUT INTO DifferentialDriveRobot.h

void ddr_callback(const geometry_msgs::Twist& msg_motor) {

	// TODO: WHAT IS THE PURPOSE OF var???

  int var = my_robot->Move(msg_motor.linear.x, msg_motor.angular.z);
}