/**
 * Base code: https://github.com/air-lasca/tutorial_controller
 *
 */

#include "ControllerServer.h"

//////////////////////////////////////////////////////////////////

ControllerServer::ControllerServer() 
	: ControllerServer::ControllerServer("pid_wheel_action_server") {}

ControllerServer::ControllerServer(std::string name):
	/** Constructor for the SimpleActionServer:
	 * 	- ros::nodeHandle n
	 *  - std::string name
	 *  - ExecuteCallback execute_cb (execute_cb, action_server, _1)
	 *  - bool auto_start
	 * 
	 */
	as(nh, "pid_wheel_control", boost::bind(&ControllerServer::executeCb, this, _1), false),
	actionName(name)
	{
		// Register callback for preemption
		as.registerPreemptCallback(boost::bind(&ControllerServer::preemptCb, this));

		// Start the server because auto_start=false
		as.start();	  

		ROS_INFO("Action Server started");
		
		// Subscriber current position of DC motor
		controlInput = nh2.subscribe("/encoder", 1, &ControllerServer::EncoderAngleCb, this);

		// Tracking control error
		pubCurrentError = nh2.advertise<robot_msgs::Motor>("/PID/control_error",1);
		
		// Publisher PID output in servo
		pubCurrentVelocity = nh2.advertise<robot_msgs::Motor>("/dc_motor", 1);
		
		// Initializing PID Controller
		Initialize();
  	}

//////////////////////////////////////////////////////////////////

void ControllerServer::EncoderAngleCb(const sensor_msgs::JointState& msg) {
	
	encoderName 	= msg.name[0];
	encoderAngle 	= msg.position[0];
	encoderVelocity = msg.velocity[0];
}

//////////////////////////////////////////////////////////////////

float ControllerServer::PIDController(float velSetPoint, float angleVelocity)
{
	double dT = (ros::Time::now() - prevTime).toSec();

	ROS_INFO("> Current velocity = %f", angleVelocity);

	// For the proportional term (also, casting angleVelocity)
	error = velSetPoint - angleVelocity;
	
	// For the integral term
	errSum += error * dT;
	errSum = std::min(errSum, MAX_CONTROL_INPUT);
	errSum = std::max(errSum, MIN_CONTROL_INPUT);

	// For the derivative term
	float dErr = (error - lastError) / dT;
	
	// Do the full calculation
	float output = (_kp * error) + (_ki * errSum) + (_kd * dErr);
   
	// Clamp output to bounds
	output = std::min(output, MAX_CONTROL_INPUT);
	output = std::max(output, MIN_CONTROL_INPUT);  

	// Required values for next round
	lastError = error;
	
	return output;
}

void ControllerServer::Initialize()
{
	ROS_INFO("Initializing PID Action Server");

  	lastError = 0;
  	errSum = 0;
    
	_kp = 1.5;
	_ki = 0.1;
	_kd = 0;
	
//	_kp = 1;
//	_ki = 2.3;
//	_kd = 0;   
}

//////////////////////////////////////////////////////////////////

//Callback for processing a goal
void ControllerServer::executeCb(const pid_wheels::PIDGoalConstPtr& goal)
{
  	prevTime = ros::Time::now();
	
	// If the server has been killed, don't process
	if(!as.isActive()||as.isPreemptRequested()) return;

	// Run the processing at 100Hz
	ros::Rate rate(100);

	// Setup some local variables
	bool success = true;	
	
	//Loop control
	while(1) {
		/*
		 * NOTE in .action file:
		 *
		 * - goal: 		float velocity, string motor
		 * - feedback: 	float velocity, string motor
		 * - result: 	bool ok
		 *
		 */

		robot_msgs::Motor VelMsg;
		
		// PID Controller
		VelMsg.name = goal->motor;
		VelMsg.data = PIDController(goal->velocity, encoderVelocity);
		
		// Publishing PID output in servo
		pubCurrentVelocity.publish(VelMsg);
		
		// Auxiliary Message
		robot_msgs::Motor ErrorMsg;
		
		ErrorMsg.name = goal->motor;
		ErrorMsg.data = error;
		
		// Publishing current error
		pubCurrentError.publish(ErrorMsg);
		
		// Filling the feedback message
		feedback.encoder = encoderName;
		feedback.angle = encoderAngle;
	    // Publish feedback to action client
	    as.publishFeedback(feedback);
		
		// Check for ROS kill
		if(!ros::ok()) {

			success = false;
			ROS_INFO("%s Shutting Down", actionName.c_str());
			break;
		}

		// If the server has been killed/preempted, stop processing
		if(!as.isActive()||as.isPreemptRequested()) return;
		
		//Sleep for rate time
		rate.sleep();
	}
	
	// Publish the result if the goal wasn't preempted
	result.ok = success;

	if(success) {

		as.setSucceeded(result);
	} else {

		as.setAborted(result,"I Failed!");
	}
}

// Callback for handling preemption. Reset your helpers here.
// Note that you still have to check for preemption in your work method to break it off
void ControllerServer::preemptCb()
{
	ROS_INFO("%s got preempted!", actionName.c_str());
	result.ok = 0;
	as.setPreempted(result, "I got Preempted!");
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pid_wheel_action_server");

	if(argc != 1) {

		ROS_INFO("Usage: pid_server");
		return 1;
	}
	
	// Spawn the server
	ControllerServer server(ros::this_node::getName());
  
	ros::spin();

	return 0;
}