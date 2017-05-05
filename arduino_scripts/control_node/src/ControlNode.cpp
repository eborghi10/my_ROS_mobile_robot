#include "DifferentialDriveRobot.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_node");

	ros::NodeHandle nh;
	DifferentialDriveRobot mobileRobot(nh);

	ros::spin();
	
	return 0;
}