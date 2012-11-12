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

#ifndef __RTT_ROS_TOOLS_THROTTLES_H
#define __RTT_ROS_TOOLS_THROTTLES_H

#include <rtt/RTT.hpp>

// Classes to throttle the rate at which something is being done in an RTT thread
  
namespace rtt_ros_tools {
  // Return TRUE when a period has expired
  class PeriodicThrottle {
  public: 
    PeriodicThrottle(RTT::os::TimeService::Seconds throttle_period);
    bool ready();
    bool ready(double throttle_period);
  private:
    RTT::os::TimeService::Seconds throttle_period_;
    RTT::os::TimeService::ticks last_time_;
  };

  // Return TRUE when a number of calls hav occurred
  class CounterThrottle {
  public: 
    CounterThrottle(size_t throttle_divider);
    bool ready();
    bool ready(size_t throttle_divider);
  private:
    size_t throttle_divider_;
    size_t loop_count_;
  };
}

#endif // ifndef __RTT_ROS_TOOLS_THROTTLES_H
