#include "PID-Wheels-Server.h"

#include <string>

ControllerServer::ControllerServer(std::string name, std::string encoder_topic):
	/** Constructor for the SimpleActionServer:
	 * 	- ros::nodeHandle n
	 *  - std::string name
	 *  - ExecuteCallback execute_cb (execute_cb, action_server, _1)
	 *  - bool auto_start
	 * 
	 */
	as(n, "pid_control", boost::bind(&ControllerServer::executeCB, this, _1), false),
	action_name(name)
	{
		// Register callback for preemption
		as.registerPreemptCallback(boost::bind(&ControllerServer::preemptCB, this));

		//Start the server because auto_start=false
		as.start();	  
		
		//Subscriber current positon of servo
		positionservosub = n2.subscribe(encoder_topic, 1, &ControllerServer::SensorCallBack, this);
		
		//Publisher setpoint, current position and error of control
		error_controlpub = n2.advertise<geometry_msgs::Vector3>(encoder_topic + "/error", 1);		
		
		//Publisher PID output in servo
		positionservopub = n2.advertise<std_msgs::Float32>("/dc_motor", 1);
		
		//Max and Min Output PID Controller (-Pi to +Pi)
		float max = M_PI;
		float min = -M_PI;
		
		//Initializing PID Controller
		Initialize(min,max);
  	}


void ControllerServer::SensorCallBack(const std_msgs::Float32& msg)
{
	/**
	 * This is a PID for controlling the position, not velocity
	 *
	 */
	position_encoder = msg.data;
}

float ControllerServer::PIDController(float setpoint, float PV)
{
	ros::Time now = ros::Time::now();
	ros::Duration change = now - prevTime;

	// For the proportional term
	float error = setpoint - PV;
	
	// For the integral term
	errSum += error * change.toSec();
	errSum = std::min(errSum, maxLimit);
	errSum = std::max(errSum, minLimit);	

	// For the derivative term
	float dErr = (error - lastError) / change.toSec();
	
	//Do the full calculation
	float output = (kp * error) + (ki * errSum) + (kd * dErr);
   
	//Clamp output to bounds
	output = std::min(output, maxLimit);
	output = std::max(output, minLimit);  

	//Required values for next round
	lastError = error;
	
	return output;
}

void ControllerServer::setOutputLimits(float min, float max)
{
	if (min > max) return;
    
	minLimit = min;
	maxLimit = max;
}

void ControllerServer::Initialize( float min, float max)
{
	setOutputLimits(min, max);
  	lastError = 0;
  	errSum = 0;
    
	kp = 1.5;
	ki = 0.1;
	kd = 0;
	
//	kp = 1;
//	ki = 2.3;
//	kd = 0;   
}


//Callback for processing a goal
void ControllerServer::executeCB(const pid_ddmr_wheels_server::PIDGoalConstPtr& goal)
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
		 * - goal: 		float32 angle, string motor
		 * - feedback: 	float32 angle, string motor
		 * - result: 	int32 ok
		 *
		 */

		std_msgs::Float32 msg_pos;
		
		//PID Controller
		msg_pos.data = PIDController(goal->angle, position_encoder);
		
		//Publishing PID output in servo
		positionservopub.publish(msg_pos);
		
		//Auxiliary Message
		geometry_msgs::Vector3 msg_error;
		
		msg_error.x = goal->angle;
		msg_error.y = position_encoder;
		msg_error.z = goal->angle - position_encoder;
		
		//Publishing setpoint, feedback and error control
		error_controlpub.publish(msg_error);
		
		feedback.angle = position_encoder;
    
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
	ControllerServer server(ros::this_node::getName(), "/encoder/left");
  
	ros::spin();

	return 0;
}