#include "ControllerClient.h"

int main (int argc, char **argv)
{
	ros::init(argc, argv, "pid_wheel_control");
	
	// create the action client
	// true causes the client to spin its own thread
	ControllerClient client(ros::this_node::getName());

	ros::spin();
	
	//exit
	return 0;
}
