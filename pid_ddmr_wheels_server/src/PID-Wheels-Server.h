#include <ros/ros.h>
#include <pid_ddmr_wheels_server/PIDAction.h>
#include <actionlib/server/simple_action_server.h>

#include "geometry_msgs/Twist.h"	// To move the motors (PWM)
#include "geometry_msgs/Vector3.h"	// Position error
#include "std_msgs/Float32.h"		// Sensor readings

#define M_PI 3.14159265358979323846

//Class for containing the server
class ControllerServer{
public:
	
	ControllerServer(std::string, std::string);

	// Callbacks
	void preemptCB();
	void executeCB(const pid_ddmr_wheels_server::PIDGoalConstPtr&);

	void Initialize(float, float);
	void setOutputLimits(float, float);
	float PIDController(float, float);
	void SensorCallBack(const std_msgs::Float32&);

protected:
	ros::NodeHandle n;
	ros::NodeHandle n2;
	
	//Subscriber
	ros::Subscriber positionservosub;
	
	//Publishers
	ros::Publisher positionservopub;
	ros::Publisher error_controlpub;
	
	//Actionlib variables
	actionlib::SimpleActionServer<pid_ddmr_wheels_server::PIDAction> as;
	pid_ddmr_wheels_server::PIDFeedback feedback;
	pid_ddmr_wheels_server::PIDResult result;
	std::string action_name;
	
	//Control variables
	float position_encoder;
	float errSum;
	float lastError;
	float minLimit, maxLimit;
	ros::Time prevTime;
	float kp;
	float ki;
	float kd;	
};