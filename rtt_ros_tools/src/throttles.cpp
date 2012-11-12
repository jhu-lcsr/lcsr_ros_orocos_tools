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

#include <rtt_ros_tools/throttles.h>

#include <rtt/RTT.hpp>



// PeriodicThrottle implementation
rtt_ros_tools::PeriodicThrottle::PeriodicThrottle(RTT::os::TimeService::Seconds throttle_period) : 
  throttle_period_(throttle_period),
  last_time_(0)
{ }

bool rtt_ros_tools::PeriodicThrottle::ready()
{
  return this->ready(throttle_period_);
}

bool rtt_ros_tools::PeriodicThrottle::ready(double throttle_period)
{
  // Check timer
  if( throttle_period_ > 0.0
      && RTT::os::TimeService::Instance()->secondsSince(last_time_) > throttle_period  )
  {
    // Store this time
    last_time_ = RTT::os::TimeService::Instance()->getTicks();
    return true;
  } 
  return false;
}

// CounterThrottle implementation
rtt_ros_tools::CounterThrottle::CounterThrottle(size_t throttle_divider) : 
  throttle_divider_(throttle_divider),
  loop_count_(0)
{ }

bool rtt_ros_tools::CounterThrottle::ready()
{
  return this->ready(throttle_divider_);
}

bool rtt_ros_tools::CounterThrottle::ready(size_t throttle_divider) {
  // Check counter
  if( throttle_divider_ > 0 && loop_count_ > throttle_divider) {
    loop_count_ = 0;
    return true;
  }
  // Increment counter
  loop_count_++;
  return false;
}
