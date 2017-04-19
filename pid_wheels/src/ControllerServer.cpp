/**
 * Base code: https://github.com/air-lasca/tutorial_controller
 *
 */

#include "ControllerServer.h"

#include <string>

/**
 * This is a PID for controlling the position, not velocity
 *
 */

//////////////////////////////////////////////////////////////////

ControllerServer::ControllerServer() :
	ControllerServer::ControllerServer("pid_wheel_control_as") {}

ControllerServer::ControllerServer(std::string name):
	/** Constructor for the SimpleActionServer:
	 * 	- ros::nodeHandle n
	 *  - std::string name
	 *  - ExecuteCallback execute_cb (execute_cb, action_server, _1)
	 *  - bool auto_start
	 * 
	 */
	as(nh, "pid_wheel_control_node", boost::bind(&ControllerServer::executeCB, this, _1), false),
	action_name(name)
	{
		// Register callback for preemption
		as.registerPreemptCallback(boost::bind(&ControllerServer::preemptCB, this));

		// Start the server because auto_start=false
		as.start();	  
		
		// Subscriber current position of DC motor
		controlInput = nh2.subscribe("/encoder", 1, &ControllerServer::EncoderAngleCb, this);

		// Tracking control error
		pubCurrentError = nh2.advertise<robot_msgs::Arduino>("/PID/control_error",1);
		
		// Publisher PID output in servo
		pubCurrentVelocity = nh2.advertise<robot_msgs::Arduino>("/dc_motor", 1);
		
		// Initializing PID Controller
		Initialize();
  	}

//////////////////////////////////////////////////////////////////

void ControllerServer::EncoderAngleCb(const robot_msgs::Arduino& msg) {
	
	encoderCb = msg.encoder;
	encoderAngle = msg.angle;
}

//////////////////////////////////////////////////////////////////

uint16_t ControllerServer::PIDController(uint16_t velSetPoint, float angleReading)
{
	double dT = (ros::Time::now() - prevTime).toSec();

	double dAngle = angleReading - lastEncoderAngle;

	double currentVelocity = dAngle / dT;

	// TODO: PRINT dAngle and casted variable

	// For the proportional term (also, casting angleReading)
	uint16_t error = velSetPoint - static_cast<uint16_t>(currentVelocity);

	// TODO: toSec() returns double!!!
	
	// For the integral term
	errSum += error * dT;
	errSum = std::min(errSum, MAX_CONTROL_INPUT);
	errSum = std::max(errSum, MIN_CONTROL_INPUT);

	// For the derivative term
	uint16_t dErr = (error - lastError) / dT;
	
	// Do the full calculation
	uint16_t output = (kp * error) + (ki * errSum) + (kd * dErr);
   
	//Clamp output to bounds
	output = std::min(output, MAX_CONTROL_INPUT);
	output = std::max(output, MIN_CONTROL_INPUT);  

	//Required values for next round
	lastError = error;
	lastEncoderAngle = angleReading;
	
	return output;
}

void ControllerServer::Initialize()
{
  	lastError = 0;
  	lastEncoderAngle = 0;
  	errSum = 0;
    
	kp = 1.5;
	ki = 0.1;
	kd = 0;
	
//	kp = 1;
//	ki = 2.3;
//	kd = 0;   
}

//////////////////////////////////////////////////////////////////

//Callback for processing a goal
void ControllerServer::executeCB(const pid_wheels::PIDGoalConstPtr& goal)
{
  	prevTime = ros::Time::now();
	
	//If the server has been killed, don't process
	if(!as.isActive()||as.isPreemptRequested()) return;

	//Run the processing at 100Hz
	ros::Rate rate(100);

	//Setup some local variables
	bool success = true;	
	
	//Loop control
	while(1)
	{
		/*
		 * NOTE in .action file:
		 *
		 * - goal: 		uint16_t angle, string motor
		 * - feedback: 	uint16_t angle, string motor
		 * - result: 	bool ok
		 *
		 */

		robot_msgs::Motor msg_pos;
		
		//PID Controller
		msg_pos.data = PIDController(goal->angle, encoderAngle);
		
		//Publishing PID output in servo
		positionservopub.publish(msg_pos);
		
		//Auxiliary Message
		geometry_msgs::Vector3 msg_error;
		
		msg_error.x = goal->angle;
		msg_error.y = encoderAngleLeft;
		msg_error.z = goal->angle - encoderAngleLeft;
		
		//Publishing setpoint, feedback and error control
		pubErrorControl.publish(msg_error);
		
		feedback.angle = encoderAngleLeft;
    
	    //Publish feedback to action client
	    as.publishFeedback(feedback);
		
		//Check for ROS kill
		if(!ros::ok())
		{
			success = false;
			ROS_INFO("%s Shutting Down", action_name.c_str());
			break;
		}

		//If the server has been killed/preempted, stop processing
		if(!as.isActive()||as.isPreemptRequested()) return;
		
		//Sleep for rate time
		rate.sleep();
	}
	
	//Publish the result if the goal wasn't preempted
	if(success)
	{
		result.ok = 1;
		as.setSucceeded(result);
	}
	else
	{
		result.ok = 0;
		as.setAborted(result,"I Failed!");
	}
}

//Callback for handling preemption. Reset your helpers here.
//Note that you still have to check for preemption in your work method to break it off
void ControllerServer::preemptCB()
{
	ROS_INFO("%s got preempted!", action_name.c_str());
	result.ok = 0;
	as.setPreempted(result, "I got Preempted!");
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pid_server");

	if(argc != 1)
	{
		ROS_INFO("Usage: pid_server");
		return 1;
	}
	
	//Spawn the server
	ControllerServer server(ros::this_node::getName());
  
	ros::spin();

	return 0;
}