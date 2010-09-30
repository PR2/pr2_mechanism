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
#include "angles/angles.h"

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


// Transmission Listener
TransmissionListener::TransmissionListener() :
  joint_name_(""),
  actuator_name_(""),
  deadband_(0.1),
  has_wrap_(false),
  has_up_(false),
  has_down_(false),
  status_(true),
  last_reading_(true)
{ }

bool TransmissionListener::initUrdf(const boost::shared_ptr<urdf::Joint> jnt, const string &actuator_name)
{
  joint_name_ = jnt->name;
  actuator_name_ = actuator_name;
  
  if (!jnt->calibration)
  {
    ROS_DEBUG("Joint \"%s\" does not support calibration.", joint_name_.c_str());
    return false;
  }

  has_up_ = (bool) jnt->calibration->rising;
  has_down_ = (bool) jnt->calibration->falling;

  if (has_up_)
    up_ref_ = *(jnt->calibration->rising);

  if (has_down_)
    down_ref_ = *(jnt->calibration->falling);
  
  if (jnt->type == urdf::Joint::CONTINUOUS)
  {
    has_wrap_ = true;

    if (has_up_ && !has_down_)
    {
      down_ref_ = up_ref_ + M_PI;
    }
    if (!has_up_ && has_down_)
      up_ref_ = down_ref_ + M_PI;

    up_ref_ = angles::normalize_angle(up_ref_);
    down_ref_ = angles::normalize_angle(down_ref_);

    has_up_ = true;
    has_down_ = true;
  }

  return true;
}

bool TransmissionListener::initParams(const ros::NodeHandle &nh)
{
  if (!nh.getParam("joint", joint_name_))
  {
    ROS_ERROR("Unable to find parameter \"joint\"");
    return false;
  }
  
  if (!nh.getParam("actuator", actuator_name_))
  {
    ROS_ERROR("Unable to find parameter \"actuator\"");
    return false;
  }
  
  if (!nh.getParam("deadband", deadband_))
  {
    ROS_ERROR("Unable to find parameter \"deadband\"");
    return false;
  }
  
  has_up_ = nh.getParam("up_ref", up_ref_);
  has_down_ = nh.getParam("down_ref", up_ref_);
  
  if (!has_up_ && !has_down_)
  {
    ROS_ERROR("Either up or down reference must be specified for joint %s", joint_name_.c_str());
    return false;
  }
    
  double tmp = 0;
  has_wrap_ = nh.getParam("wrap", tmp);
  
  if (has_wrap_ && (!has_up_ || !has_down_))
  {
    ROS_ERROR("For continuous joints, both \"up_ref\" and \"down_ref\" must be specified. Error on joint %s", 
              joint_name_.c_str());
    return false;
  }
  
  if (!has_wrap_ && (!nh.hasParam("max") || !nh.hasParam("min")))
  {
    ROS_ERROR("For non-continuous joints, either \"max\" and \"min\" must be specified. Error on joint %s", 
              joint_name_.c_str());
    return false;
  }
  
  nh.getParam("max", max_);
  nh.getParam("min", min_);

  return true;
}

// Look for joint, actuator
// Check bounds
bool TransmissionListener::update(const pr2_mechanism_msgs::MechanismStatistics::ConstPtr& mechMsg)
{
  const pr2_mechanism_msgs::JointStatistics *js = NULL;
  const pr2_mechanism_msgs::ActuatorStatistics *as = NULL;

  for (uint i = 0; i < mechMsg->joint_statistics.size(); ++i)
  {
    if (mechMsg->joint_statistics[i].name == joint_name_)
      js = &mechMsg->joint_statistics[i];
  }

  if (!js)
  {
    ROS_ERROR_ONCE("Unable to find joint state for joint \"%s\".", joint_name_.c_str());
    return false;
  }

  if (!js->is_calibrated)
    return true; // Ignore until we're calibrated

  // Same for actuators
  for (uint i = 0; i < mechMsg->actuator_statistics.size(); ++i)
  {
    if (mechMsg->actuator_statistics[i].name == actuator_name_)
      as = &mechMsg->actuator_statistics[i];
  }

  if (!as)
  {
    ROS_ERROR_ONCE("Unable to find actuator state for actuator \"%s\".", actuator_name_.c_str());
    return false;
  }

  /*
  if (!checkBounds(js))
  {
    last_reading_ = false;
    status_ = false;
    return false;
  }
  */

  bool rv = checkFlag(js, as);

  last_reading_ = rv;
  if (!rv)
    status_ = false;

  return rv;
}

bool TransmissionListener::checkBounds(const pr2_mechanism_msgs::JointStatistics *js) const
{
  ROS_ASSERT_MSG(js->name == joint_name_, "Joint name didn't match!");

  // We should only see calibrated joints
  if (!js->is_calibrated)
  {
    ROS_ERROR_ONCE("Joint \"%s\" isn't calibrated. Unable to check bounds for joint.", joint_name_.c_str());
    return false; 
  }

  return js->position < max_ && js->position > min_;
}

bool TransmissionListener::checkFlag(const pr2_mechanism_msgs::JointStatistics *js,
                                     const pr2_mechanism_msgs::ActuatorStatistics *as) const
{
  ROS_ASSERT_MSG(js->name == joint_name_, "Joint name didn't match!");
  ROS_ASSERT_MSG(as->name == actuator_name_, "Actuator name didn't match!");

  float jnt_position = js->position;

  if (!has_up_ && !has_down_)
  {
    ROS_ERROR("Neither up or down reference specified for joint \"%s\". Unable to check transmission.",
              joint_name_.c_str());
    return false;
  }

  // Check if we need to wrap position
  if (has_wrap_)
    jnt_position = angles::normalize_angle(jnt_position);

  // Check deadband for up reference
  if (has_up_)
  {
    if (abs(jnt_position - up_ref_) < deadband_)
      return true; // Too close to deadband
    if (abs((2 * M_PI - jnt_position) - up_ref_) < deadband_)
      return true; // Close to DB on other side of wrap
  }

  // Check deadband for down ref
  if (has_down_)
  {
    if (abs(jnt_position - down_ref_) < deadband_)
      return true; // Too close to deadband
    if (abs((2 * M_PI - jnt_position) - down_ref_) < deadband_)
      return true; // Close to DB on other side of wrap
  }
  
  if (has_up_ && has_down_)
  {
    if (up_ref_ > down_ref_)
    {
      if (jnt_position < down_ref_ && as->calibration_reading)
        return true;
      else if (jnt_position > down_ref_ && jnt_position < up_ref_ && !as->calibration_reading)
        return true;
      else if (jnt_position > up_ref_ && as->calibration_reading)
        return true;
      else 
      {
        ROS_WARN_THROTTLE(1, "Broken transmission reading for \"%s\". Position: %f (wrapped), cal reading: %d. Up ref: %f, Down ref: %f", joint_name_.c_str(), jnt_position, as->calibration_reading, up_ref_, down_ref_);
        return false;
      }
    }
    else // down_ref_ > up_ref_
    {
      if (jnt_position < up_ref_ && !as->calibration_reading)
        return true;
      else if (jnt_position > up_ref_ && jnt_position < down_ref_ && as->calibration_reading)
        return true;
      else if (jnt_position > down_ref_ && !as->calibration_reading)
        return true;
      else 
      {
        ROS_WARN_THROTTLE(1, "Broken transmission reading for \"%s\". Position: %f (wrapped), cal reading: %d. Up ref: %f, Down ref: %f", joint_name_.c_str(), jnt_position, as->calibration_reading, up_ref_, down_ref_);
        return false;
      }
    }
  }
  else if (has_up_)
  {
    if (jnt_position > up_ref_ && as->calibration_reading)
      return true;
    else if (jnt_position < up_ref_ && !as->calibration_reading)
      return true;
    else
    {
      ROS_WARN_THROTTLE(1, "Broken transmission reading for \"%s\". Position: %f (wrapped), cal reading: %d. Up ref: %f, Down ref: %f", joint_name_.c_str(), jnt_position, as->calibration_reading, up_ref_, down_ref_);
        return false;
    }
  }

  else // has_down_
  {
    if (jnt_position > down_ref_ && !as->calibration_reading)
      return true;
    else if (jnt_position < down_ref_ && as->calibration_reading)
      return true;
    else
    {
      ROS_WARN_THROTTLE(1, "Broken transmission reading for \"%s\". Position: %f (wrapped), cal reading: %d. Up ref: %f, Down ref: %f", joint_name_.c_str(), jnt_position, as->calibration_reading, up_ref_, down_ref_);
        return false;
    }
  }
    
  ROS_ASSERT_MSG(false, "No case handled for transmission checker!");
  return false;
}

boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> TransmissionListener::toDiagStat() const
{
  boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> stat(new diagnostic_updater::DiagnosticStatusWrapper);
  
  stat->name = "Transmission (" + joint_name_ + ")";
      
  if (!status_)
    stat->mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Broken Transmission");

  stat->add("Transmission Status", status_ ? "OK" : "Broken");
  stat->add("Current Reading", last_reading_ ? "OK" : "Broken");
  
  stat->add("Joint", joint_name_);
  stat->add("Actuator", actuator_name_);
  if (has_up_)
    stat->add("Up Position", up_ref_);
  else
    stat->add("Up Position", "N/A");

  if (has_down_)
    stat->add("Down Position", down_ref_);
  else
    stat->add("Down Position", "N/A");

  stat->add("Wrapped", has_wrap_ ? "True" : "False");
  stat->add("Max Limit", max_);
  stat->add("Min Limit", min_);
 
  // Todo: Start tracking state....
  
  return stat;
}



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

