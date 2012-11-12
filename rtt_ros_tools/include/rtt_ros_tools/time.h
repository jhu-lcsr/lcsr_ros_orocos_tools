/*
 * Copyright (c) 2012, The Johns Hopkins University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The Johns Hopkins University. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __RTT_ROS_TOOLS_TIME_H
#define __RTT_ROS_TOOLS_TIME_H

#include <rtt/RTT.hpp>
#include <ros/ros.h>

namespace rtt_ros_tools {

  // Function to get a ros::Time initialized from RTT::os::TimeService
  ros::Time ros_rt_now();

  // Class for picking either ROS time or RT time
  class TimeLord {
  public:
    typedef enum {
      UNKNOWN = 0,
      WALL_TIME,
      SIM_TIME
    } time_state_t;

    static void Init() {
      if( TimeState_ == UNKNOWN ) {
        // Check if ROS is using simulation time
        ros::NodeHandle nh;
        bool use_sim_time = false;

        use_sim_time = nh.getParam("use_sim_time",use_sim_time) && use_sim_time;

        if(use_sim_time) {
          TimeState_ = SIM_TIME;
        } else {
          TimeState_ = WALL_TIME;
        }
      }
    }

    static ros::Time autotime_now() {
      TimeLord::Init();

      if(TimeState_ == SIM_TIME) {
        // Use the ROS clock time, which in this case will be the time from the /clock topic
        return ros::Time::now();
      } else {
        // Use NTP-synchronized RTC time in an RT system, and the ROS clock time in a non-rt system
        return rtt_ros_tools::ros_rt_now();
      }
    }

    static ros::Time rostime_now() {
      return ros::Time::now();
    }

    static ros::Time walltime_now() {
      return ros_rt_now();
    }

  protected:
    static time_state_t TimeState_;
  };
}

#endif // ifndef __RTT_ROS_TOOLS_TIME_H
