/**
 *  http://wiki.ros.org/actionlib#Using_the_ActionClient
 *
 */

#include <ros/ros.h>
// #include <package_name/FileNameAction.h>
// package_name/action/FileName.action
#include <pid_wheels/PIDAction.h>
// actionlib::SimpleActionServer API
#include <actionlib/server/simple_action_server.h>

#include "geometry_msgs/Twist.h"	// To move the motors (PWM)
#include "geometry_msgs/Vector3.h"	// Position error
#include "std_msgs/Float32.h"		// Sensor readings

#define M_PI 3.14159265358979323846

// Max and Min Output PID Controller (-Pi to +Pi)
const uint16_t MAX_CONTROL_INPUT = 0xFFFF;
const uint16_t MIN_CONTROL_INPUT = 0;


//Class for containing the server
class ControllerServer{
public:
	// Constructor
	ControllerServer();
	ControllerServer(std::string);

	// Callbacks
	void preemptCB();
	void executeCB(const pid_wheels::PIDGoalConstPtr&);

	void Initialize();
	void setOutputLimits(float, float);
	float PIDController(float, float);
	// Subscribers callbacks
	void PositionLeftCb(const std_msgs::Float32&);
	void PositionRightCb(const std_msgs::Float32&);

protected:
	ros::NodeHandle nh;
	ros::NodeHandle nh2;
	
	//Subscriber
	ros::Subscriber controlInput;
	
	//Publishers
	ros::Publisher positionservopub;
	ros::Publisher pubErrorControl;
	
	//Actionlib variables
	actionlib::SimpleActionServer<pid_wheels::PIDAction> as;
	pid_wheels::PIDFeedback feedback;
	pid_wheels::PIDResult result;
	std::string action_name;
	
	/*
	 * Control variables
	 *
	 */
	// Encoder readings (control input)
	float encoderAngle;
	std::string encoderCb;
	// PID parameters
	float lastEncoderAngle;
	float errSum;
	float lastError;
	ros::Time prevTime;
	float kp;
	float ki;
	float kd;	
};