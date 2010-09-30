/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include "pr2_mechanism_diagnostics/controller_diagnostics.h"


using namespace pr2_mechanism_diagnostics;
using namespace std;

// Controller statistics
ControllerStats::ControllerStats(string nam, bool disable_warnings) :
  name(nam),
  timestamp(0),
  running(false),
  num_overruns(0),
  last_overrun_time(0),
  disable_warnings_(disable_warnings)
{ }

bool ControllerStats::update(const pr2_mechanism_msgs::ControllerStatistics &cs)
{
  if (name != cs.name)
  {
    ROS_ERROR("Controller statistics attempted to update with a different name! Old name: %s, new name: %s.", name.c_str(), cs.name.c_str());
    return false;
  }
  
  timestamp     = cs.timestamp;
  running       = cs.running;
  max_time      = cs.max_time;
  mean_time     = cs.mean_time;
  variance_time = cs.variance_time;
  num_overruns  = cs.num_control_loop_overruns;
  last_overrun_time = cs.time_last_control_loop_overrun;
  
  updateTime = ros::Time::now();
  
  return true;
}

boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> ControllerStats::toDiagStat() const
{
  boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> stat(new diagnostic_updater::DiagnosticStatusWrapper);
  
  stat->name = "Controller (" + name + ")";
  
  if (running)
    stat->summary(diagnostic_msgs::DiagnosticStatus::OK, "Running");
  else
    stat->summary(diagnostic_msgs::DiagnosticStatus::OK, "Stopped");
  
  if (!disable_warnings_ && num_overruns > 0)
  {
    if ((ros::Time::now() - last_overrun_time).toSec() < 30)
      stat->summary(diagnostic_msgs::DiagnosticStatus::WARN, "!!! Broke Realtime, used more than 1000 micro seconds in update loop");
    else
      stat->summary(diagnostic_msgs::DiagnosticStatus::OK, "!!! Broke Realtime, used more than 1000 micro seconds in update loop");
  }
  
  stat->add("Avg Update Time (usec)", (int)(mean_time.toSec() * 1e6));
  stat->add("Max Update Time (usec)", (int)(max_time.toSec() * 1e6));
  stat->add("Variance Update Time (usec)", (int) (variance_time.toSec() * 1e6));
  stat->add("Percent of Cycle Time Used", (int) (mean_time.toSec() / 0.00001));
  stat->add("Number of Control Loop Overruns", num_overruns);
  stat->add("Timestamp of Last Overrun (sec)", last_overrun_time.toSec());
  
  return stat;
}

