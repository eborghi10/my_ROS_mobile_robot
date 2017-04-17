#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "PIDActionGoal.h"
#include "PIDActionResult.h"
#include "PIDActionFeedback.h"

namespace action_controller
{
	class PID : public ros::Msg
	{
	public:
		typedef action_controller::PIDActionGoal _action_goal_type;
		_action_goal_type action_goal;
		
		typedef action_controller::PIDActionResult _action_result_type;
		_action_result_type action_result;
		
		typedef action_controller::PIDActionFeedback _action_feedback_type;
		_action_feedback_type action_feedback;	

		PID(): 
			action_goal(),
		  	action_result(),
		  	action_feedback() {}

		virtual int serialize(unsigned char *outbuffer) const
	    {
	      int offset = 0;
	      offset += this->action_goal.serialize(outbuffer + offset);
	      offset += this->action_result.serialize(outbuffer + offset);
	      offset += this->action_feedback.serialize(outbuffer + offset);
	      return offset;
	    }

	    virtual int deserialize(unsigned char *inbuffer)
	    {
	      int offset = 0;
	      offset += this->action_goal.deserialize(inbuffer + offset);
	      offset += this->action_result.deserialize(inbuffer + offset);
	      offset += this->action_feedback.deserialize(inbuffer + offset);
	     return offset;
	    }

	    const char * getType(){ return "include/action_controller/PIDAction"; };
	    /**
	     * MD5 generated via http://www.miraclesalad.com/webtools/md5.php
	     *
	     */
	    const char * getMD5(){ return "88dfe7f9982362d9dc239635746f4e56"; };
	};
}
