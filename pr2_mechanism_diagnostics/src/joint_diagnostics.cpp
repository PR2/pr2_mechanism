/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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


#include "pr2_mechanism_diagnostics/joint_diagnostics.h"
#include <limits>

#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

using namespace pr2_mechanism_diagnostics;
using namespace std;

/*** TODO
 * Set deadband properly for each joint
 * Figure out max/min
 * Track state with accumulators
 * Only halt motors if we've had >3 consecutive hits, etc
 * Add tests for warnings
 * Publish status topic (bool) at 1 Hz
 * 
 *****/



// Joint statistics
JointStats::JointStats(string nam) : 
  needs_reset(true),
  name(nam),
  position(0),
  velocity(0),
  measured_effort(0),
  commanded_effort(0),
  is_calibrated(false),
  violated_limits(false),
  odometer(0),
  max_pos_val(-1 * numeric_limits<double>::max()),
  min_pos_val(numeric_limits<double>::max()),
  max_abs_vel_val(-1 * numeric_limits<double>::max()),
  max_abs_eff_val(-1 * numeric_limits<double>::max())    
{ }

void JointStats::reset_vals()
{
  needs_reset = false;
  
  max_pos_val = -1 * numeric_limits<double>::max();
  min_pos_val = numeric_limits<double>::max();
  max_abs_vel_val = -1 * numeric_limits<double>::max();
  max_abs_eff_val = -1 * numeric_limits<double>::max();
}


boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> JointStats::toDiagStat() const
{
  boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> stat(new diagnostic_updater::DiagnosticStatusWrapper);
  
  stat->name = "Joint (" + name + ")";
  
  if (is_calibrated)
    stat->summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  else
    stat->summary(diagnostic_msgs::DiagnosticStatus::WARN, "Uncalibrated");
  
  if ((ros::Time::now() - updateTime).toSec() > 3)
    stat->summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No updates");
  
  if (!is_valid(position) || !is_valid(velocity) || 
      !is_valid(measured_effort) || !is_valid(commanded_effort))
  {
    stat->summary(diagnostic_msgs::DiagnosticStatus::ERROR, "NaN Values in Joint State");
  }
  
  stat->add("Position", position);
  stat->add("Velocity", velocity);
  stat->add("Measured Effort", measured_effort);
  stat->add("Commanded Effort", commanded_effort);
  
  stat->add("Calibrated", is_calibrated);
  
  stat->add("Odometer", odometer);
  
  if (is_calibrated)
  {
    stat->add("Max Position", max_pos_val);
    stat->add("Min Position", min_pos_val);
    stat->add("Max Abs. Velocity", max_abs_vel_val);
    stat->add("Max Abs. Effort", max_abs_eff_val);
  }
  else
  {
    stat->add("Max Position", "N/A");
    stat->add("Min Position", "N/A");
    stat->add("Max Abs. Velocity", "N/A");
    stat->add("Max Abs. Effort", "N/A");
  }
  
  stat->add("Limits Hit", violated_limits);
  
  needs_reset = true;
  
  return stat;
}

bool JointStats::update(const pr2_mechanism_msgs::JointStatistics &js)
{
  if (name != js.name)
  {
    ROS_ERROR("Joint statistics attempted to update with a different name! Old name: %s, new name: %s.", name.c_str(), js.name.c_str());
    return false;
  }
  
  if (needs_reset)
    reset_vals();
  
  if (js.is_calibrated)
  {
    max_pos_val     = max(max_pos_val,     js.max_position);
    min_pos_val     = min(max_pos_val,     js.min_position);
    max_abs_vel_val = max(max_abs_vel_val, js.max_abs_velocity);
    max_abs_eff_val = max(max_abs_eff_val, js.max_abs_effort);
  }
  
  position         = js.position;
  velocity         = js.velocity;
  measured_effort  = js.measured_effort;
  commanded_effort = js.commanded_effort;
  is_calibrated    = js.is_calibrated;
  violated_limits  = js.violated_limits;
  odometer         = js.odometer;
  
  updateTime = ros::Time::now();
  
  return true;
}

