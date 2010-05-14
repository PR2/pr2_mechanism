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

///\author Kevin Watts
///\brief Publishes diagnostics for controllers, joints

#ifndef _PR2_MECHANISM_DIAGNOSTICS_H_
#define _PR2_MECHANISM_DIAGNOSTICS_H_

#include <ros/ros.h>
#include <pr2_mechanism_msgs/MechanismStatistics.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <vector>
#include <float.h>
#include <limits>
#include <boost/shared_ptr.hpp>

namespace pr2_mechanism_diagnostics
{

///\todo Switch to boost::math::isnormal()
///\brief Returns true if value is NaN
template<typename T>
inline bool isNan(T value)
{
  return value != value;
}

class ControllerStats
{
private:
  ros::Time updateTime;
  
  std::string name;
  ros::Time timestamp;
  bool running;
  ros::Duration max_time;
  ros::Duration mean_time;
  ros::Duration variance_time;
  int num_overruns;
  ros::Time last_overrun_time;

  bool disable_warnings_;

public:
  ControllerStats(bool disable_warnings) :
    name(""),
    timestamp(0),
    running(false),
    num_overruns(0),
    last_overrun_time(0),
    disable_warnings_(disable_warnings)
  { }
    
  ~ControllerStats() {}

  bool update(const pr2_mechanism_msgs::ControllerStatistics &cs)
  {
    if (name == "")
      name = cs.name;

    if (name != cs.name)
    {
      ROS_ERROR("Controller statistics was updated with a different name! Old name: %s, new name: %s.", name.c_str(), cs.name.c_str());
      return false;
    }

    timestamp = cs.timestamp;
    running = cs.running;
    max_time = cs.max_time;
    mean_time = cs.mean_time;
    variance_time = cs.variance_time;
    num_overruns = cs.num_control_loop_overruns;
    last_overrun_time = cs.time_last_control_loop_overrun;

    updateTime = ros::Time::now();

    return true;
  }

  bool shouldDiscard() const { return (ros::Time::now() - updateTime).toSec() > 3; }


  boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> toDiagStat()
  {
    boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> stat(new diagnostic_updater::DiagnosticStatusWrapper);

    stat->name = "Controller (" + name + ")";

    if (running)
      stat->summary(diagnostic_msgs::DiagnosticStatus::OK, "Running");
    else
      stat->summary(diagnostic_msgs::DiagnosticStatus::OK, "Stopped");

    if (!disable_warnings_ && num_overruns > 0)
    {
      stat->summary(diagnostic_msgs::DiagnosticStatus::WARN, "!!! Broke Realtime, used more than 1000 micro seconds in update loop");
    }
       
    stat->add("Avg Update Time (usec)", (int)(mean_time.toSec() * 1e6));
    stat->add("Max Update Time (usec)", (int)(max_time.toSec() * 1e6));
    stat->add("Variance Update Time (usec)", (int) (variance_time.toSec() * 1e6));
    stat->add("Percent of Cycle Time Used", (int) (mean_time.toSec() / 0.00001));
    stat->add("Number of Control Loop Overruns", num_overruns);
    stat->add("Timestamp of Last Overrun (sec)", last_overrun_time.toSec());

    return stat;
  }

};

class JointStats 
{
private:
  ros::Time updateTime;

  bool needs_reset;

  std::string name;
  double position;
  double velocity;
  double measured_effort;
  double commanded_effort;
  bool is_calibrated;
  bool violated_limits;
  double odometer;

  double max_pos_val, min_pos_val, max_abs_vel_val, max_abs_eff_val;

  void reset_vals()
  {
    needs_reset = false;
    
    max_pos_val = -1 * std::numeric_limits<double>::max();
    min_pos_val = std::numeric_limits<double>::max();
    max_abs_vel_val = -1 * std::numeric_limits<double>::max();
    max_abs_eff_val = -1 * std::numeric_limits<double>::max();
  }
  
public:
  JointStats() : 
    needs_reset(true),
    name(""),
    position(0),
    velocity(0),
    measured_effort(0),
    commanded_effort(0),
    is_calibrated(false),
    violated_limits(false),
    odometer(0),
    max_pos_val(-1 * std::numeric_limits<double>::max()),
    min_pos_val(std::numeric_limits<double>::max()),
    max_abs_vel_val(-1 * std::numeric_limits<double>::max()),
    max_abs_eff_val(-1 * std::numeric_limits<double>::max())    
  { }

  ~JointStats() { }

  bool update(const pr2_mechanism_msgs::JointStatistics &js)
  {
    if (name == "")
      name = js.name;
      

    if (name != js.name)
    {
      ROS_ERROR("Joint statistics was updated with a different name! Old name: %s, new name: %s.", name.c_str(), js.name.c_str());
      return false;
    }

    if (needs_reset)
      reset_vals();

    max_pos_val     = std::max(max_pos_val, js.max_position);
    min_pos_val     = std::min(max_pos_val, js.min_position);
    max_abs_vel_val = std::max(max_abs_vel_val, js.max_abs_velocity);
    max_abs_eff_val = std::max(max_abs_eff_val, js.max_abs_effort);

    position = js.position;
    velocity = js.velocity;
    measured_effort = js.measured_effort;
    commanded_effort = js.commanded_effort;
    is_calibrated = js.is_calibrated;
    violated_limits = js.violated_limits;
    odometer = js.odometer;
    
    updateTime = ros::Time::now();
    
    return true;
  }

  boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> toDiagStat()
  {
    boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> stat(new diagnostic_updater::DiagnosticStatusWrapper);

    stat->name = "Joint (" + name + ")";

    if (is_calibrated)
      stat->summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    else
      stat->summary(diagnostic_msgs::DiagnosticStatus::WARN, "Uncalibrated");
    
    double updateSec = (ros::Time::now() - updateTime).toSec();
    if (updateSec > 3)
      stat->summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No updates");
    
    if (isNan(position) || isNan(velocity) || isNan(measured_effort) || isNan(commanded_effort))
      stat->summary(diagnostic_msgs::DiagnosticStatus::ERROR, "NaN Values");
    
    stat->add("Position", position);
    stat->add("Velocity", velocity);
    stat->add("Measured Effort", measured_effort);
    stat->add("Commanded Effort", commanded_effort);

    if (is_calibrated)
      stat->add("Calibrated", "True");
    else
      stat->add("Calibrated", "False");

    stat->add("Odometer", odometer);
    stat->add("Max Position", max_pos_val);
    stat->add("Min Position", min_pos_val);
    stat->add("Max Abs. Velocity", max_abs_vel_val);
    stat->add("Max Abs. Effort", max_abs_eff_val);

    if (violated_limits)
      stat->add("Limits Hit", "True");
    else
      stat->add("Limits Hit", "False");

    needs_reset = true;

    return stat;
  }
};

class CtrlJntDiagnosticPublisher
{
private:
  std::map<std::string, JointStats*> joint_stats;

  std::map<std::string, ControllerStats *> controller_stats;

  ros::NodeHandle n_;
  ros::Subscriber mech_st_sub_;
  ros::Publisher diag_pub_;

  bool use_sim_time_;
  bool disable_controller_warnings_;
  
  void mechCallback(const pr2_mechanism_msgs::MechanismStatistics::ConstPtr& mechMsg)
  {
    std::vector<pr2_mechanism_msgs::JointStatistics>::const_iterator it;
    for (it = mechMsg->joint_statistics.begin(); it != mechMsg->joint_statistics.end(); ++it)
    {
      if (joint_stats.count(it->name) == 0)
        joint_stats[it->name] = new JointStats();

      joint_stats[it->name]->update(*it);
    }

    std::vector<pr2_mechanism_msgs::ControllerStatistics>::const_iterator c_it;
    for (c_it = mechMsg->controller_statistics.begin(); c_it != mechMsg->controller_statistics.end(); ++c_it)
    {
      if (controller_stats.count(c_it->name) == 0)
        controller_stats[c_it->name] = new ControllerStats(use_sim_time_ || disable_controller_warnings_);
      
      controller_stats[c_it->name]->update(*c_it);
    }
  }

public:
  CtrlJntDiagnosticPublisher() :
    use_sim_time_(false),
    disable_controller_warnings_(false)
  { 
    mech_st_sub_ = n_.subscribe("mechanism_statistics", 1000, &CtrlJntDiagnosticPublisher::mechCallback, this);
    diag_pub_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    n_.param("use_sim_time", use_sim_time_, false);
    n_.param("disable_controller_warnings", disable_controller_warnings_, false);
  }

  ~CtrlJntDiagnosticPublisher() 
  {
    std::map<std::string, JointStats*>::iterator it;
    for (it = joint_stats.begin(); it != joint_stats.end(); ++it)
      delete it->second;

    std::map<std::string, ControllerStats*>::iterator c_it;
    for (c_it = controller_stats.begin(); c_it != controller_stats.end(); ++c_it)
      delete c_it->second;
  }

  void publishDiag()
  {
    diagnostic_msgs::DiagnosticArray array;
    
    std::map<std::string, JointStats*>::iterator it;
    for (it = joint_stats.begin(); it != joint_stats.end(); ++it)
      array.status.push_back(*(it->second->toDiagStat()));

    std::map<std::string, ControllerStats*>::iterator c_it;
    unsigned int num_controllers = 0;
    for (c_it = controller_stats.begin(); c_it != controller_stats.end(); ++c_it)
    {
      if (c_it->second->shouldDiscard())
        controller_stats.erase(c_it);

      array.status.push_back(*(c_it->second->toDiagStat()));
      num_controllers++;
    }
    if (num_controllers == 0){
      diagnostic_updater::DiagnosticStatusWrapper stat;
      stat.name = "Controller: No controllers loaded in controller manager";
      array.status.push_back(stat);
    }

    array.header.stamp = ros::Time::now();

    diag_pub_.publish(array);

  }

  bool ok() const { return n_.ok(); }

};

}


#endif
