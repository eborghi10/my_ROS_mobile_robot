#pragma once

//////////////////////////////////////////////////////////////////

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include "DCMotor.h"

//////////////////////////////////////////////////////////////////

class ActionBridge
{
  ros::NodeHandle *nh;
public:
  ActionBridge(ros::NodeHandle&);

  void SendGoal(INT_PWM);
  void CancelGoal();

  virtual void FeedbackLeftCb(const std_msgs::UInt16&) = 0;
  virtual void FeedbackRightCb(const std_msgs::UInt16&) = 0;
  void ResultCb(const std_msgs::Bool&);

  ros::Publisher pub_goal;
  ros::Publisher pub_cancel;

  ros::Subscriber<std_msgs::UInt16, ActionBridge> sub_left_feedback;
  ros::Subscriber<std_msgs::UInt16, ActionBridge> sub_right_feedback;
  ros::Subscriber<std_msgs::Bool, ActionBridge> sub_result;

  std_msgs::UInt16 goal_msg;
  std_msgs::Empty cancel_msg;
  std_msgs::Bool result_msg;
};

//////////////////////////////////////////////////////////////////

ActionBridge::ActionBridge(ros::NodeHandle &nh)
  : nh(&nh),
    pub_goal("/bridge/goal", &goal_msg),
    pub_cancel("/bridge/cancel", &cancel_msg),
    sub_left_feedback("/bridge/feedback/left", &ActionBridge::FeedbackLeftCb, this),
    sub_right_feedback("/bridge/feedback/right", &ActionBridge::FeedbackRightCb, this),
    sub_result("/bridge/result", &ActionBridge::ResultCb, this) {

    nh.advertise(pub_goal);
    nh.advertise(pub_cancel);
    nh.subscribe(sub_left_feedback);
    nh.subscribe(sub_right_feedback);
    nh.subscribe(sub_result);
  }

//////////////////////////////////////////////////////////////////

void ActionBridge::SendGoal(INT_PWM goal) {

  goal_msg.data = goal;
  pub_goal.publish(&goal_msg);
}

void ActionBridge::CancelGoal(void) {

  pub_cancel.publish(&cancel_msg);
}

//////////////////////////////////////////////////////////////////

void ActionBridge::ResultCb(const std_msgs::Bool& result_msg) {

  char* str = "[ActionBridge] ";
  snprintf(str,sizeof(str) + sizeof(std_msgs::Bool),"%d",result_msg.data);
  nh->loginfo(str);
}
