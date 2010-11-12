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
#include "pr2_mechanism_model/robot.h"
#include "pr2_mechanism_diagnostics/joint_diagnostics.h"
#include "pr2_mechanism_diagnostics/controller_diagnostics.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"

namespace pr2_mechanism_diagnostics
{

/**
 * Publishes diagnostics data from pr2_mechanism_msgs/MechanismStatistics
 * for joints and controllers.
 */
class CtrlJntDiagnosticPublisher
{
private:
  std::map<std::string, boost::shared_ptr<JointStats> > joint_stats;
  std::map<std::string, boost::shared_ptr<ControllerStats> > controller_stats;

  ros::NodeHandle n_, pnh_;
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
