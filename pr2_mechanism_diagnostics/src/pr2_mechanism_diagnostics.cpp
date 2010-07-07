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


#include "pr2_mechanism_diagnostics/pr2_mechanism_diagnostics.h"
#include <exception>
#include <limits>

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

// Diagnostic publisher
CtrlJntDiagnosticPublisher::CtrlJntDiagnosticPublisher() :
  use_sim_time_(false),
  disable_controller_warnings_(false)
{ 
  mech_st_sub_ = n_.subscribe("mechanism_statistics", 1000, &CtrlJntDiagnosticPublisher::mechCallback, this);
  diag_pub_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
  
  n_.param("/use_sim_time", use_sim_time_, false);

  ros::NodeHandle pnh("~");
  pnh.param("disable_controller_warnings", disable_controller_warnings_, false);
}


void CtrlJntDiagnosticPublisher::mechCallback(const pr2_mechanism_msgs::MechanismStatistics::ConstPtr& mechMsg)
{
  // Update joints
  vector<pr2_mechanism_msgs::JointStatistics>::const_iterator it;
  for (it = mechMsg->joint_statistics.begin(); it != mechMsg->joint_statistics.end(); ++it)
  {
    if (joint_stats.count(it->name) == 0)
      joint_stats[it->name] = boost::shared_ptr<JointStats>(new JointStats(it->name));
    
    joint_stats[it->name]->update(*it);
  }
  
  // Update controllers
  vector<pr2_mechanism_msgs::ControllerStatistics>::const_iterator c_it;
  for (c_it = mechMsg->controller_statistics.begin(); c_it != mechMsg->controller_statistics.end(); ++c_it)
  {
    if (controller_stats.count(c_it->name) == 0)
      controller_stats[c_it->name] = boost::shared_ptr<ControllerStats>(new ControllerStats(c_it->name, use_sim_time_ || disable_controller_warnings_));
    
    controller_stats[c_it->name]->update(*c_it);
  }
}

void CtrlJntDiagnosticPublisher::publishDiag()
{
  diagnostic_msgs::DiagnosticArray array;
  
  // Update joints
  map<string, boost::shared_ptr<JointStats> >::iterator it;
  for (it = joint_stats.begin(); it != joint_stats.end(); ++it)
    array.status.push_back(*(it->second->toDiagStat()));
  
  // Update controllers. Note controllers that haven't update are discarded
  map<string, boost::shared_ptr<ControllerStats> >::iterator c_it;
  unsigned int num_controllers = 0;
  vector<string> erase_controllers;
  for (c_it = controller_stats.begin(); c_it != controller_stats.end(); ++c_it)
  {
    if (c_it->second->shouldDiscard())
      erase_controllers.push_back(c_it->first);
    
    array.status.push_back(*(c_it->second->toDiagStat()));
    num_controllers++;
  }
  for (unsigned int i=0; i<erase_controllers.size(); i++)
    controller_stats.erase(erase_controllers[i]);
  
  // Publish even if we have no controllers loaded
  if (num_controllers == 0)
  {
    diagnostic_updater::DiagnosticStatusWrapper stat;
    stat.name = "Controller: No controllers loaded in controller manager";
    array.status.push_back(stat);
  }
  
  array.header.stamp = ros::Time::now();
  diag_pub_.publish(array);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_mechanism_diagnostics");
  
  try
  {
    CtrlJntDiagnosticPublisher ctrlJntPub;
    
    ros::Rate pub_rate(1.0);
    while (ctrlJntPub.ok())
    {
      pub_rate.sleep();
      ros::spinOnce();
      ctrlJntPub.publishDiag();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("Controllers/Joint to Diagnostics node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }
  
  exit(0);
  return 0;
}
