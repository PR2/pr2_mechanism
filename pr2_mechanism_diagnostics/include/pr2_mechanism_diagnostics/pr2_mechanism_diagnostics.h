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

#ifndef _PR2_MECHANISM_DIAGNOSTICS_H_MECH_DIAG_
#define _PR2_MECHANISM_DIAGNOSTICS_H_MECH_DIAG_

#include <ros/ros.h>
#include <pr2_mechanism_msgs/MechanismStatistics.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <vector>
#include <float.h>
#include <limits>
#include <boost/shared_ptr.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

namespace pr2_mechanism_diagnostics
{

template<typename T>
inline bool is_valid(T t)
{
  if (t == 0)
    return true;

  return boost::math::isnormal(t);
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

  bool update(const pr2_mechanism_msgs::ControllerStatistics &cs);

  bool shouldDiscard() const { return (ros::Time::now() - updateTime).toSec() > 3; }


  boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> toDiagStat();

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

  bool update(const pr2_mechanism_msgs::JointStatistics &js);

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

};

class CtrlJntDiagnosticPublisher
{
private:
  std::map<std::string, boost::shared_ptr<JointStats> > joint_stats;

  std::map<std::string, boost::shared_ptr<ControllerStats> > controller_stats;

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
        joint_stats[it->name] = boost::shared_ptr<JointStats>(new JointStats());

      joint_stats[it->name]->update(*it);
    }

    std::vector<pr2_mechanism_msgs::ControllerStatistics>::const_iterator c_it;
    for (c_it = mechMsg->controller_statistics.begin(); c_it != mechMsg->controller_statistics.end(); ++c_it)
    {
      if (controller_stats.count(c_it->name) == 0)
        controller_stats[c_it->name] = boost::shared_ptr<ControllerStats>(new ControllerStats(use_sim_time_ || disable_controller_warnings_));
      
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

  ~CtrlJntDiagnosticPublisher() { }

  void publishDiag()
  {
    diagnostic_msgs::DiagnosticArray array;
    
    std::map<std::string, boost::shared_ptr<JointStats> >::iterator it;
    for (it = joint_stats.begin(); it != joint_stats.end(); ++it)
      array.status.push_back(*(it->second->toDiagStat()));

    std::map<std::string, boost::shared_ptr<ControllerStats> >::iterator c_it;
    unsigned int num_controllers = 0;
    std::vector<std::string> erase_controllers;
    for (c_it = controller_stats.begin(); c_it != controller_stats.end(); ++c_it)
    {
      if (c_it->second->shouldDiscard())
	erase_controllers.push_back(c_it->first);

      array.status.push_back(*(c_it->second->toDiagStat()));
      num_controllers++;
    }
    for (unsigned int i=0; i<erase_controllers.size(); i++)
      controller_stats.erase(erase_controllers[i]);

    //\todo Fix unit test to stop this
    //if (num_controllers == 0){
    //  diagnostic_updater::DiagnosticStatusWrapper stat;
    //  stat.name = "Controller: No controllers loaded in controller manager";
    //   array.status.push_back(stat);
    // }

    array.header.stamp = ros::Time::now();

    diag_pub_.publish(array);

  }

  bool ok() const { return n_.ok(); }
};

}


#endif
