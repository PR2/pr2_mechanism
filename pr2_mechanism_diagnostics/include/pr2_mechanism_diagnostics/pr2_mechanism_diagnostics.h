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
///\brief Publishes diagnostics for controllers, joints from pr2_mechanism_msgs/MechanismStatistics message

#ifndef _PR2_MECHANISM_DIAGNOSTICS_H_MECH_DIAG_
#define _PR2_MECHANISM_DIAGNOSTICS_H_MECH_DIAG_

#include <ros/ros.h>
#include <pr2_mechanism_msgs/MechanismStatistics.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <vector>
#include <float.h>
#include <boost/shared_ptr.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

namespace pr2_mechanism_diagnostics
{

/**< \brief Returns false if a value is infinity, NaN, etc */
template<typename T>
inline bool is_valid(T t)
{
  if (t == 0)
    return true;

  return boost::math::isnormal(t);
}

/**
 * Tracks data from controllers and publishes to diagnostics
 * Updates with pr2_mechanism_msgs/ControllerStatistics data
 * 
 * Controllers that don't update in more than 3 seconds will be discarded.
 * Controllers that have overran the 1000us limit will report a warning
 * for the 30 seconds after the most recent overrun.
 */
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
  ControllerStats(std::string nam, bool disable_warnings); 
    
  ~ControllerStats() {}

  bool update(const pr2_mechanism_msgs::ControllerStatistics &cs);

  /**<! True if we should discard stale controller value */
  bool shouldDiscard() const { return (ros::Time::now() - updateTime).toSec() > 3; }

  boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> toDiagStat() const;

};

/** 
 * Tracks data from joints and publishes to diagnostics
 * Updates with pr2_mechanism_msgs/JointStatistics data
 * 
 * Any joint that reports a NaN or Inf value will report an error.
 * Any joint that is uncalibrated will report a warning.
 */
class JointStats 
{
private:
  ros::Time updateTime;

  mutable bool needs_reset;

  std::string name;
  double position;
  double velocity;
  double measured_effort;
  double commanded_effort;
  bool is_calibrated;
  bool violated_limits;
  double odometer;

  // Store min/max positions, etc for joint since last diagnostic publish
  double max_pos_val, min_pos_val, max_abs_vel_val, max_abs_eff_val;

  void reset_vals(); /**!< Resets min/max values for next publish */
 
  
public:
  JointStats(std::string nam);

  ~JointStats() { }

  bool update(const pr2_mechanism_msgs::JointStatistics &js);

  boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> toDiagStat() const;
};

/**
 * Publishes diagnostics data from pr2_mechanism_msgs/MechanismStatistics
 * for joints and controllers.
 */
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
  
  void mechCallback(const pr2_mechanism_msgs::MechanismStatistics::ConstPtr& mechMsg);

public:
  CtrlJntDiagnosticPublisher(); 

  ~CtrlJntDiagnosticPublisher() { }

  void publishDiag(); /**< Publish diagnostics from joints, controllers */

  bool ok() const { return n_.ok(); } 
};

}

#endif // _PR2_MECHANISM_DIAGNOSTICS_H_MECH_DIAG_
