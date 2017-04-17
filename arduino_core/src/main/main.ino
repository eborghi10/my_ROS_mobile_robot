#include "include/DifferentialDriveRobot.h"

//////////////////////////////////////////////////////////////////

DifferentialDriveRobot *my_robot;

//////////////////////////////////////////////////////////////////

void setup()
{
  my_robot = new DifferentialDriveRobot(
    new DCMotor(IN1, IN2, new MagneticEncoder(CS1)),
    new DCMotor(IN3, IN4, new MagneticEncoder(CS2)),
    "/encoder/left", "/encoder/right"
  );

  my_robot->UpdatePhysicalParameters(0.2, 1.0);
}

//////////////////////////////////////////////////////////////////

void loop() 
{
	(my_robot->nh).logdebug("Initializing program...");
	(my_robot->nh).spinOnce();

  my_robot->SendAngles();

  if( !(my_robot->nh).connected() )  my_robot->Stop();
}