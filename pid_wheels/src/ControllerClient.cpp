#include "ControllerClient.h"
#include "DifferentialDriveRobot.h"

int main (int argc, char **argv)
{
	ros::init(argc, argv, "pid_wheel_control");
	
	// create the action client
	// true causes the client to spin its own thread
	ControllerClient client(ros::this_node::getName());

	DifferentialDriveRobot mobileRobot();

	ros::spin();
	
	return 0;
}
