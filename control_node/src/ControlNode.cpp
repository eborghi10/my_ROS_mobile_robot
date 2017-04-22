#include "DifferentialDriveRobot.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "control_node");

	DifferentialDriveRobot mobileRobot();

	ros::spin();
	
	return 0;
}