///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008-2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Stuart Glaser, Wim Meeussen
 */
#ifndef MECHANISM_CONTROL_H
#define MECHANISM_CONTROL_H

#include <pthread.h>
#include <map>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "pr2_controller_manager/controller_spec.h"
#include <tinyxml.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/robot.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <pr2_controller_interface/controller_provider.h>
#include "pluginlib/class_loader.h"
#include <pr2_mechanism_msgs/ListControllerTypes.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <pr2_mechanism_msgs/ReloadControllerLibraries.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/MechanismStatistics.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread/condition.hpp>


namespace pr2_controller_manager{

class ControllerManager: public pr2_controller_interface::ControllerProvider {

public:
  ControllerManager(pr2_hardware_interface::HardwareInterface *hw, 
                   const ros::NodeHandle& nh=ros::NodeHandle());
  virtual ~ControllerManager();

  // Real-time functions
  void update();

  // Non real-time functions
  bool initXml(TiXmlElement* config);
  bool loadController(const std::string& name);
  bool unloadController(const std::string &name);
  bool switchController(const std::vector<std::string>& start_controllers,
                        const std::vector<std::string>& stop_controllers,
                        const int strictness);

  // controllers_lock_ must be locked before calling
  virtual pr2_controller_interface::Controller* getControllerByName(const std::string& name);

  pr2_mechanism_model::Robot model_;
  pr2_mechanism_model::RobotState *state_;

private:
  void getControllerNames(std::vector<std::string> &v);
  void getControllerSchedule(std::vector<size_t> &schedule);

  ros::NodeHandle controller_node_, cm_node_;
  boost::shared_ptr<pluginlib::ClassLoader<pr2_controller_interface::Controller> > controller_loader_;

  // for controller switching
  std::vector<pr2_controller_interface::Controller*> start_request_, stop_request_;
  bool please_switch_;
  int switch_strictness_;

  // controller lists
  boost::mutex controllers_lock_;
  std::vector<ControllerSpec> controllers_lists_[2];
  std::vector<size_t>   controllers_scheduling_[2];
  int current_controllers_list_, used_by_realtime_;

  // for controller statistics
  Statistics pre_update_stats_;
  Statistics update_stats_;
  Statistics post_update_stats_;

  // for publishing constroller state
  void publishJointState();
  void publishMechanismStatistics();
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub_joint_state_;
  realtime_tools::RealtimePublisher<pr2_mechanism_msgs::MechanismStatistics> pub_mech_stats_;
  ros::Duration publish_period_joint_state_, publish_period_mechanism_stats_;
  ros::Time last_published_joint_state_, last_published_mechanism_stats_;

  // services to work with controllers
  bool listControllerTypesSrv(pr2_mechanism_msgs::ListControllerTypes::Request &req,
                              pr2_mechanism_msgs::ListControllerTypes::Response &resp);
  bool listControllersSrv(pr2_mechanism_msgs::ListControllers::Request &req,
                          pr2_mechanism_msgs::ListControllers::Response &resp);
  bool switchControllerSrv(pr2_mechanism_msgs::SwitchController::Request &req,
                           pr2_mechanism_msgs::SwitchController::Response &resp);
  bool loadControllerSrv(pr2_mechanism_msgs::LoadController::Request &req,
                          pr2_mechanism_msgs::LoadController::Response &resp);
  bool unloadControllerSrv(pr2_mechanism_msgs::UnloadController::Request &req,
                         pr2_mechanism_msgs::UnloadController::Response &resp);
  bool reloadControllerLibrariesSrv(pr2_mechanism_msgs::ReloadControllerLibraries::Request &req,
                                    pr2_mechanism_msgs::ReloadControllerLibraries::Response &resp);
  boost::mutex services_lock_;
  ros::ServiceServer srv_list_controllers_, srv_list_controller_types_, srv_load_controller_;
  ros::ServiceServer srv_unload_controller_, srv_switch_controller_, srv_reload_libraries_;
  bool motors_previously_halted_;
};

}
#endif /* MECHANISM_CONTROL_H */
