#include <ros/ros.h>
#include <pid_wheels/PIDAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float32.h"

//class containing the client
class ControllerClient{

  public:
    ControllerClient(std::string name);

    void doneCb(const actionlib::SimpleClientGoalState&, const pid_wheels::PIDResultConstPtr&);
    void activeCb();
    void feedbackCb(const pid_wheels::PIDFeedbackConstPtr&);
    void GoalCallback(const std_msgs::Float32&);

private:
	actionlib::SimpleActionClient<pid_wheels::PIDAction> ac;
	std::string action_name;	
	pid_wheels::PIDGoal goal;	
	ros::Subscriber goalsub;
	ros::NodeHandle n;
};


ControllerClient::ControllerClient(std::string name):

	    //Set up the client. It's publishing to topic "pid_control", and is set to auto-spin
	    ac("pid_control", true),
	    //Stores the name
	    action_name(name)
	    {
	      //Get connection to a server
	      ROS_INFO("%s Waiting For Server...", action_name.c_str());

	      //Wait for the connection to be valid
	      ac.waitForServer();

	      ROS_INFO("%s Got a Server...", action_name.c_str());
	
	      goalsub = n.subscribe("/dc_motor", 100, &ControllerClient::GoalCallback, this);
      }

// Called once when the goal completes
void ControllerClient::doneCb(const actionlib::SimpleClientGoalState& state, const pid_wheels::PIDResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());

	ROS_INFO("Result: %i", result->ok);
}

// Called once when the goal becomes active
void ControllerClient::activeCb()
{
	ROS_INFO("Goal just went active...");
}

// Called every time feedback is received for the goal
void ControllerClient::feedbackCb(const pid_wheels::PIDFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback of Progress to Goal: position: %f", feedback->angle);
}

void ControllerClient::GoalCallback(const std_msgs::Float32& msg)
{	
	goal.angle = msg.data;
	
	ac.sendGoal(
		goal, 
		boost::bind(&ControllerClient::doneCb, this, _1, _2),
		boost::bind(&ControllerClient::activeCb, this),
		boost::bind(&ControllerClient::feedbackCb, this, _1)
	);
}