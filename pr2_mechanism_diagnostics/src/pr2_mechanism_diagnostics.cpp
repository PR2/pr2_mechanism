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
#include "angles/angles.h"

using namespace pr2_mechanism_diagnostics;
using namespace std;

// Diagnostic publisher
CtrlJntDiagnosticPublisher::CtrlJntDiagnosticPublisher() :
  pnh_("~"),
  use_sim_time_(false),
  disable_controller_warnings_(false)
{ 
  mech_st_sub_ = n_.subscribe("mechanism_statistics", 1000, &CtrlJntDiagnosticPublisher::mechCallback, this);
  diag_pub_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
  
  n_.param("/use_sim_time", use_sim_time_, false);

  pnh_.param("disable_controller_warnings", disable_controller_warnings_, false);
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
