#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace action_controller
{
  class PIDGoal : public ros::Msg
  {
    public:
      	typedef int32_t _samples_type;
      	_samples_type samples;

    PIDGoal():
      	samples(0) {}

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_samples;
      u_samples.real = this->samples;
      *(outbuffer + offset + 0) = (u_samples.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_samples.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_samples.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_samples.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->samples);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_samples;
      u_samples.base = 0;
      u_samples.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_samples.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_samples.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_samples.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->samples = u_samples.real;
      offset += sizeof(this->samples);
     return offset;
    }

    const char * getType(){ return "action_controller/PIDGoal"; };
    const char * getMD5(){ return "9ae37b8daaa0962064c3b09ec5b6cee6"; };
  };
}