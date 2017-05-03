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

#include <robot_msgs/Motor.h>
#include <sensor_msgs/JointState.h>
#include <string>

#define M_PI 3.14159265358979323846

// Max and Min Output PID Controller (-Pi to +Pi)
const float MAX_CONTROL_INPUT = 500;
const float MIN_CONTROL_INPUT = 0;


// Class for containing the server
class ControllerServer{
public:
	// Constructor
	ControllerServer();
	ControllerServer(std::string);

	// Callbacks
	void preemptCb();
	void executeCb(const pid_wheels::PIDGoalConstPtr&);

	void Initialize();
	float PIDController(float, float);
	// Subscribers callbacks
	void EncoderAngleCb(const sensor_msgs::JointState&);

protected:
	ros::NodeHandle nh;		// Action Server handle
	ros::NodeHandle nh2;	// Topics handle
	
	// Subscriber
	ros::Subscriber controlInput;
	
	// Publishers
	ros::Publisher pubCurrentVelocity;
	ros::Publisher pubCurrentError;
	
	//Actionlib variables
	actionlib::SimpleActionServer<pid_wheels::PIDAction> as;
	pid_wheels::PIDFeedback feedback;
	pid_wheels::PIDResult result;
	std::string actionName;
	
	/*
	 * Control variables
	 *
	 */
	// Encoder readings (control input)
	float encoderAngle;
	float encoderVelocity;
	std::string encoderName;
	// PID parameters
	float error;
	float errSum;
	float lastError;
	ros::Time prevTime;
	float _kp;
	float _ki;
	float _kd;	
};