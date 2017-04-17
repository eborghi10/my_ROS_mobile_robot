#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "action_msgs/GoalStatus.h"
#include "PIDFeedback.h"

namespace action_controller 
{
	class PIDActionFeedback : public ros::Msg
	{
	public:
		  typedef std_msgs::Header _header_type;
      _header_type header;
      typedef action_msgs::GoalStatus _status_type;
      _status_type status;
      typedef action_controller::PIDFeedback _feedback_type;
      _feedback_type feedback;

    PIDActionFeedback():
    	 header(),
      	status(),
      	feedback() {}

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "action_controller/PIDActionFeedback"; };
    const char * getMD5(){ return "3f6bb3fa6f4627b576230ea218361de7"; };
	};
}