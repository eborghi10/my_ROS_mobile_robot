#include <ros/ros.h>
#include <pid_wheels/PIDAction.h>
#include <actionlib/client/simple_action_client.h>

#include <robot_msgs/Arduino.h>

//class containing the client
class ControllerClient{

  public:
  	ControllerClient();
    ControllerClient(std::string name);

private:
	actionlib::SimpleActionClient<pid_wheels::PIDAction> ac;
	std::string actionName;	
	pid_wheels::PIDGoal goal;

	ros::NodeHandle nh;

	void doneCb(const actionlib::SimpleClientGoalState&, const pid_wheels::PIDResultConstPtr&);
    void activeCb();
    void feedbackCb(const pid_wheels::PIDFeedbackConstPtr&);

    void SetGoal(const robot_msgs::Arduino&);
};

ControllerClient::ControllerClient()
	: ControllerClient::ControllerClient("pid_wheel_control") {}

ControllerClient::ControllerClient(std::string name):

	    // Set up the client and is set to auto-spin
	    ac("pid_wheel_control", true),
	    //Stores the name
	    actionName(name)
	    {
	      // Get connection to a server
	      ROS_INFO("%s Waiting For Server...", actionName.c_str());

	      //Wait for the connection to be valid
	      ac.waitForServer();

	      ROS_INFO("%s Got a Server...", actionName.c_str());
      }

// Called once when the goal completes
void ControllerClient::doneCb(const actionlib::SimpleClientGoalState& state, const pid_wheels::PIDResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());

	ROS_INFO("Result: %d", result->ok);
}

// Called once when the goal becomes active
void ControllerClient::activeCb()
{
	ROS_INFO("Goal just went active...");
}

// Called every time feedback is received for the goal
void ControllerClient::feedbackCb(const pid_wheels::PIDFeedbackConstPtr& feedback)
{
	ROS_INFO("feedback [%s]: %f", (feedback->encoder).c_str(), feedback->angle);
}

/////////////////////////////////////////////////////////////////

void ControllerClient::SetGoal(const robot_msgs::Arduino& msg) {

	goal.motor = msg.name;
	goal.velocity = msg.data;

	ac.sendGoal(
		goal,
		boost::bind(&ControllerClient::doneCb, this, _1, _2),
		boost::bind(&ControllerClient::activeCb, this),
		boost::bind(&ControllerClient::feedbackCb, this, _1)
	);
}