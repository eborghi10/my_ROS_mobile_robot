#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "action_msgs/GoalID.h"
#include "PIDGoal.h"

namespace action_controller
{

  class PIDActionGoal : public ros::Msg
  {
    public:
    	typedef std_msgs::Header _header_type;
      	_header_type header;
      	typedef action_msgs::GoalID _goal_id_type;
      	_goal_id_type goal_id;
      	typedef action_controller::PIDGoal _goal_type;
      	_goal_type goal;

    PIDActionGoal():
    	header(),
      	goal_id(),
      	goal() {}

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "action_controller/PIDActionGoal"; };
    const char * getMD5(){ return "43e1cf03d0115a0af1072b61047236df"; };
  };
}