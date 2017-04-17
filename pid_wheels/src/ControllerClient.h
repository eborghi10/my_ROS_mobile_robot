#include <ros/ros.h>
#include <pid_wheels/PIDAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

//class containing the client
class ControllerClient{

  public:
    ControllerClient(std::string name);

    void doneCb(const actionlib::SimpleClientGoalState&, const pid_wheels::PIDResultConstPtr&);
    void activeCb();
    void feedbackCb(const pid_wheels::PIDFeedbackConstPtr&);

    void GoalBridgeCb(const std_msgs::UInt16&);
    void CancelBridgeCb(const std_msgs::Empty&);

private:
	actionlib::SimpleActionClient<pid_wheels::PIDAction> ac;
	std::string action_name;	
	pid_wheels::PIDGoal goal;	
	ros::NodeHandle nh;

	ros::Subscriber sub_goal;
	ros::Subscriber sub_cancel;
	ros::Publisher pub_left_feedback;
	ros::Publisher pub_right_feedback;
	ros::Publisher pub_result;
};

ControllerClient::ControllerClient(std::string name):

	    // Set up the client. It's publishing to topic "pid_control", 
		// and is set to auto-spin
	    ac("pid_control", true),
	    //Stores the name
	    action_name(name)
	    {
	      //Get connection to a server
	      ROS_INFO("%s Waiting For Server...", action_name.c_str());

	      //Wait for the connection to be valid
	      ac.waitForServer();

	      ROS_INFO("%s Got a Server...", action_name.c_str());

	      sub_goal = nh.subscribe("/bridge/goal", 5, &ControllerClient::GoalBridgeCb, this);
	      sub_cancel = nh.subscribe("/bridge/cancel", 5, &ControllerClient::CancelBridgeCb, this);

	      pub_left_feedback = nh.advertise<std_msgs::UInt16>("/bridge/feedback/left", 5);
	      pub_right_feedback = nh.advertise<std_msgs::UInt16>("/bridge/feedback/right", 5);
	      pub_result = nh.advertise<std_msgs::Bool>("/bridge/result", 5);
      }

// Called once when the goal completes
void ControllerClient::doneCb(const actionlib::SimpleClientGoalState& state, const pid_wheels::PIDResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());

	ROS_INFO("Result: %d", result->ok);

	pub_result.publish(result->ok);
}

// Called once when the goal becomes active
void ControllerClient::activeCb()
{
	ROS_INFO("Goal just went active...");
}

// Called every time feedback is received for the goal
void ControllerClient::feedbackCb(const pid_wheels::PIDFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback of Progress to Goal: position: %d", feedback->angle);

	if (strcmp((feedback->motor).c_str(),"Left") == 0)
	{
		pub_left_feedback.publish(feedback->angle);
	} else {

		pub_right_feedback.publish(feedback->angle);
	}
}

void ControllerClient::GoalBridgeCb(const std_msgs::UInt16& msg) {

	goal.angle = msg.data;

	ac.sendGoal(
		goal,
		boost::bind(&ControllerClient::doneCb, this, _1, _2),
		boost::bind(&ControllerClient::activeCb, this),
		boost::bind(&ControllerClient::feedbackCb, this, _1)
	);
}

void ControllerClient::CancelBridgeCb(const std_msgs::Empty& msg) {

	// Cancel all previous goals
	ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
}