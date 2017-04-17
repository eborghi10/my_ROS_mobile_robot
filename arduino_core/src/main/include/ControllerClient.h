#pragma once

#include <ros.h>
#include "action_controller/PIDAction.h"
//#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float32.h"

//class containing the client
class ControllerClient{

  public:
  	ControllerClient();
    ControllerClient(char* name);

    void doneCb(const actionlib::SimpleClientGoalState&, const action_controller::PIDResultConstPtr&);
    void activeCb();
    void feedbackCb(const action_controller::PIDFeedbackConstPtr&);
    void GoalCallback(const std_msgs::Float32&);

private:
	actionlib::SimpleActionClient<action_controller::PIDAction> ac;
	char* action_name;	
	action_controller::PIDGoal goal;	
	ros::Subscriber <std_msgs::Float32, ControllerClient> goalsub;
	ros::NodeHandle nh;
};

ControllerClient::ControllerClient():
	//Set up the client. It's publishing to topic "pid_control", and is set to auto-spin
    ac("pid_control", true),
    //Stores the name
    action_name("controller_action_client"),
    // subscribe
    sub("/dc_motor", &ControllerClient::GoalCallback, this)
    {
      //Get connection to a server
      nh.loginfo("%s Waiting For Server...", action_name.c_str());

      //Wait for the connection to be valid
      ac.waitForServer();

      nh.loginfo("%s Got a Server...", action_name.c_str());

      nh.subscribe(sub);
  	}

ControllerClient::ControllerClient(char* name):
	//Stores the name
    action_name(name),
    ControllerClient::ControllerClient() {}
	    

// Called once when the goal completes
void ControllerClient::doneCb(const actionlib::SimpleClientGoalState& state, const action_controller::PIDResultConstPtr& result)
{
	nh.loginfo("Finished in state [%s]", state.toString().c_str());

	nh.loginfo("Result: %i", result->ok);
}

// Called once when the goal becomes active
void ControllerClient::activeCb()
{
	nh.loginfo("Goal just went active...");
}

// Called every time feedback is received for the goal
void ControllerClient::feedbackCb(const action_controller::PIDFeedbackConstPtr& feedback)
{
	nh.loginfo("Got Feedback of Progress to Goal: position: %f", feedback->angle);
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