#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "action_msgs/GoalStatus.h"
#include "PIDResult.h"

namespace action_controller
{
  class PIDActionResult : public ros::Msg
  {
    public:
    	typedef std_msgs::Header _header_type;
      	_header_type header;
      	typedef action_msgs::GoalStatus _status_type;
      	_status_type status;
      	typedef action_controller::PIDResult _result_type;
      	_result_type result;

    PIDActionResult():
    	header(),
      	status(),
      	result() {}

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "action_controller/PIDActionResult"; };
    const char * getMD5(){ return "9a54b3d1d4b254e02245ccbbcb5f9529"; };
  };
}