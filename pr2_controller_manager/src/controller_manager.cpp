////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include "pr2_controller_manager/controller_manager.h"
#include "pr2_controller_manager/scheduler.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include "ros/console.h"

using namespace pr2_mechanism_model;
using namespace pr2_controller_manager;
using namespace pr2_hardware_interface;
using namespace pr2_controller_interface;
using namespace boost::accumulators;
using namespace ros;


ControllerManager::ControllerManager(HardwareInterface *hw, const ros::NodeHandle& nh) :
  model_(hw),
  state_(NULL),
  controller_node_(nh),
  cm_node_(nh, "pr2_controller_manager"),
  start_request_(0),
  stop_request_(0),
  please_switch_(false),
  current_controllers_list_(0),
  used_by_realtime_(-1),
  pub_joint_state_(nh, "joint_states", 1),
  pub_mech_stats_(nh, "mechanism_statistics", 1),
  last_published_joint_state_(ros::Time::now()),
  last_published_mechanism_stats_(ros::Time::now())
{}

ControllerManager::~ControllerManager()
{
  if (state_)
    delete state_;
}


bool ControllerManager::initXml(TiXmlElement* config)
{
  if (!model_.initXml(config)){
    ROS_ERROR("Failed to initialize pr2 mechanism model");
    return false;
  }
  state_ = new RobotState(&model_);
  motors_previously_halted_ = state_->isHalted();

  // pre-allocate for realtime publishing
  pub_mech_stats_.msg_.controller_statistics.resize(0);
  pub_mech_stats_.msg_.actuator_statistics.resize(model_.hw_->actuators_.size());
  int joints_size = 0;
  for (unsigned int i = 0; i < state_->joint_states_.size(); ++i)
  {
    int type = state_->joint_states_[i].joint_->type;
    if (type != urdf::Joint::REVOLUTE &&
        type != urdf::Joint::CONTINUOUS &&
        type != urdf::Joint::PRISMATIC){
      ROS_ERROR("Joint state contains joint '%s' of unknown type", state_->joint_states_[i].joint_->name.c_str());
      return false;
    }
    ++joints_size;
  }
  pub_mech_stats_.msg_.joint_statistics.resize(joints_size);
  pub_joint_state_.msg_.name.resize(joints_size);
  pub_joint_state_.msg_.position.resize(joints_size);
  pub_joint_state_.msg_.velocity.resize(joints_size);
  pub_joint_state_.msg_.effort.resize(joints_size);

  // get the publish rate for mechanism state
  double publish_rate_joint_state, publish_rate_mechanism_stats;
  cm_node_.param("mechanism_statistics_publish_rate", publish_rate_mechanism_stats, 1.0);
  cm_node_.param("joint_state_publish_rate", publish_rate_joint_state, 100.0);
  publish_period_mechanism_stats_ = Duration(1.0/fmax(0.000001, publish_rate_mechanism_stats));
  publish_period_joint_state_ = Duration(1.0/fmax(0.000001, publish_rate_joint_state));

  // create controller loader
  controller_loader_.reset(new pluginlib::ClassLoader<pr2_controller_interface::Controller>("pr2_controller_interface",
                                                                                           "pr2_controller_interface::Controller"));
  // Advertise services (this should be the last thing we do in init)
  srv_list_controllers_ = cm_node_.advertiseService("list_controllers", &ControllerManager::listControllersSrv, this);
  srv_list_controller_types_ = cm_node_.advertiseService("list_controller_types", &ControllerManager::listControllerTypesSrv, this);
  srv_load_controller_ = cm_node_.advertiseService("load_controller", &ControllerManager::loadControllerSrv, this);
  srv_unload_controller_ = cm_node_.advertiseService("unload_controller", &ControllerManager::unloadControllerSrv, this);
  srv_switch_controller_ = cm_node_.advertiseService("switch_controller", &ControllerManager::switchControllerSrv, this);
  srv_reload_libraries_ = cm_node_.advertiseService("reload_controller_libraries", &ControllerManager::reloadControllerLibrariesSrv, this);

  return true;
}





// Must be realtime safe.
void ControllerManager::update()
{
  used_by_realtime_ = current_controllers_list_;
  std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];
  std::vector<size_t> &scheduling = controllers_scheduling_[used_by_realtime_];

  ros::Time start = ros::Time::now();
  state_->propagateActuatorPositionToJointPosition();
  state_->zeroCommands();
  ros::Time start_update = ros::Time::now();
  pre_update_stats_.acc((start_update - start).toSec());

  // Restart all running controllers if motors are re-enabled
  if (!state_->isHalted() && motors_previously_halted_){
    for (size_t i=0; i<controllers.size(); i++){
      if (controllers[scheduling[i]].c->isRunning()){
        controllers[scheduling[i]].c->stopRequest();
        controllers[scheduling[i]].c->startRequest();
      }
    }
  }
  motors_previously_halted_ = state_->isHalted();

  // Update all controllers in scheduling order
  for (size_t i=0; i<controllers.size(); i++){
    ros::Time start = ros::Time::now();
    controllers[scheduling[i]].c->updateRequest();
    ros::Time end = ros::Time::now();
    controllers[scheduling[i]].stats->acc((end - start).toSec());
    if (end - start > ros::Duration(0.001)){
      controllers[scheduling[i]].stats->num_control_loop_overruns++;
      controllers[scheduling[i]].stats->time_last_control_loop_overrun = end;
    }
  }
  ros::Time end_update = ros::Time::now();
  update_stats_.acc((end_update - start_update).toSec());

  state_->enforceSafety();
  state_->propagateJointEffortToActuatorEffort();
  ros::Time end = ros::Time::now();
  post_update_stats_.acc((end - end_update).toSec());

  // publish state
  publishMechanismStatistics();
  publishJointState();

  // there are controllers to start/stop
  if (please_switch_)
  {
    // stop controllers
    for (unsigned int i=0; i<stop_request_.size(); i++)
      if (!stop_request_[i]->stopRequest())
        ROS_FATAL("Failed to stop controller in realtime loop. This should never happen.");

    // start controllers
    for (unsigned int i=0; i<start_request_.size(); i++)
      if (!start_request_[i]->startRequest())
        ROS_FATAL("Failed to start controller in realtime loop. This should never happen.");

    start_request_.clear();
    stop_request_.clear();
    please_switch_ = false;
  }
}

pr2_controller_interface::Controller* ControllerManager::getControllerByName(const std::string& name)
{
  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    if (controllers[i].name == name)
      return controllers[i].c.get();
  }
  return NULL;
}

void ControllerManager::getControllerNames(std::vector<std::string> &names)
{
  boost::mutex::scoped_lock guard(controllers_lock_);
  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    names.push_back(controllers[i].name);
  }
}

void ControllerManager::getControllerSchedule(std::vector<size_t> &schedule)
{
  boost::mutex::scoped_lock guard(controllers_lock_);
  schedule = controllers_scheduling_[current_controllers_list_];
}


bool ControllerManager::loadController(const std::string& name)
{
  ROS_DEBUG("Will load controller '%s'", name.c_str());

  // lock controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  // get reference to controller list
  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (ros::ok() && free_controllers_list == used_by_realtime_){
    if (!ros::ok())
      return false;
    usleep(200);
  }
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // Copy all controllers from the 'from' list to the 'to' list
  for (size_t i = 0; i < from.size(); ++i)
    to.push_back(from[i]);

  // Checks that we're not duplicating controllers
  for (size_t j = 0; j < to.size(); ++j)
  {
    if (to[j].name == name)
    {
      to.clear();
      ROS_ERROR("A controller named '%s' was already loaded inside the controller manager", name.c_str());
      return false;
    }
  }

  NodeHandle c_node;
  // Constructs the controller
  try{
    c_node = NodeHandle(controller_node_, name);
  }
  catch(std::exception &e) {
    ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s':\n%s", name.c_str(), e.what());
    return false;
  }
  catch(...){
    ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s'", name.c_str());
    return false;
  }
  boost::shared_ptr<pr2_controller_interface::Controller> controller;
  std::string type;
  if (c_node.getParam("type", type))
  {
    ROS_DEBUG("Constructing controller '%s' of type '%s'", name.c_str(), type.c_str());
    try {
      // Backwards compatibility for using non-namespaced controller types
      if (!controller_loader_->isClassAvailable(type))
      {
        std::vector<std::string> classes = controller_loader_->getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i)
        {
          if(type == controller_loader_->getName(classes[i]))
          {
            ROS_WARN("The deprecated controller type %s was not found.  Using the namespaced version %s instead.  "
                     "Please update your configuration to use the namespaced version.",
                     type.c_str(), classes[i].c_str());
            type = classes[i];
            break;
          }
        }
      }

      controller = controller_loader_->createInstance(type);
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
    }
  }

  // checks if controller was constructed
  if (controller == NULL)
  {
    to.clear();
    if (type == "")
      ROS_ERROR("Could not load controller '%s' because the type was not specified. Did you load the controller configuration on the parameter server?",
              name.c_str());
    else
      ROS_ERROR("Could not load controller '%s' because controller type '%s' does not exist",
                name.c_str(), type.c_str());
    return false;
  }

  // Initializes the controller
  ROS_DEBUG("Initializing controller '%s'", name.c_str());
  bool initialized;
  try{
    initialized = controller->initRequest(this, state_, c_node);
  }
  catch(std::exception &e){
    ROS_ERROR("Exception thrown while initializing controller %s.\n%s", name.c_str(), e.what());
    initialized = false;
  }
  catch(...){
    ROS_ERROR("Exception thrown while initializing controller %s", name.c_str());
    initialized = false;
  }
  if (!initialized)
  {
    to.clear();
    ROS_ERROR("Initializing controller '%s' failed", name.c_str());
    return false;
  }
  ROS_DEBUG("Initialized controller '%s' succesful", name.c_str());

  // Adds the controller to the new list
  to.resize(to.size() + 1);
  to[to.size()-1].name = name;
  to[to.size()-1].c = controller;

  //  Do the controller scheduling
  if (!scheduleControllers(to, controllers_scheduling_[free_controllers_list])){
    to.clear();
    ROS_ERROR("Scheduling controller '%s' failed", name.c_str());
    return false;
  }

  // Resize controller state vector
  pub_mech_stats_.lock();
  pub_mech_stats_.msg_.controller_statistics.resize(to.size());
  for (size_t i=0; i<to.size(); i++)
    pub_mech_stats_.msg_.controller_statistics[i].name = to[i].name;

  // Destroys the old controllers list when the realtime thread is finished with it.
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;
  while (ros::ok() && used_by_realtime_ == former_current_controllers_list_){
    if (!ros::ok())
      return false;
    usleep(200);
  }
  from.clear();
  pub_mech_stats_.unlock();

  ROS_DEBUG("Successfully load controller '%s'", name.c_str());
  return true;
}




bool ControllerManager::unloadController(const std::string &name)
{
  ROS_DEBUG("Will unload controller '%s'", name.c_str());

  // lock the controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  // get reference to controller list
  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (ros::ok() && free_controllers_list == used_by_realtime_){
    if (!ros::ok())
      return false;
    usleep(200);
  }
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // check if no other controller depends on this controller
  for (size_t i = 0; i < from.size(); ++i){
    for (size_t b=0; b<from[i].c->before_list_.size(); b++){
      if (name == from[i].c->before_list_[b]){
        ROS_ERROR("Cannot unload controller %s because controller %s still depends on it",
                  name.c_str(), from[i].name.c_str());
        return false;
      }
    }
    for (size_t a=0; a<from[i].c->after_list_.size(); a++){
      if (name == from[i].c->after_list_[a]){
        ROS_ERROR("Cannot unload controller %s because controller %s still depends on it",
                  name.c_str(), from[i].name.c_str());
        return false;
      }
    }
  }

  // Transfers the running controllers over, skipping the one to be removed and the running ones.
  bool removed = false;
  for (size_t i = 0; i < from.size(); ++i)
  {
    if (from[i].name == name){
      if (from[i].c->isRunning()){
        to.clear();
        ROS_ERROR("Could not unload controller with name %s because it is still running",
                  name.c_str());
        return false;
      }
      removed = true;
    }
    else
      to.push_back(from[i]);
  }

  // Fails if we could not remove the controllers
  if (!removed)
  {
    to.clear();
    ROS_ERROR("Could not unload controller with name %s because no controller with this name exists",
              name.c_str());
    return false;
  }

  //  Do the controller scheduling
  ROS_DEBUG("Rescheduling controller execution order");
  if (!scheduleControllers(to, controllers_scheduling_[free_controllers_list])){
    to.clear();
    ROS_ERROR("Scheduling controllers failed when removing controller '%s' failed", name.c_str());
    return false;
  }

  // Resize controller state vector
  ROS_DEBUG("Resizing controller state vector");
  pub_mech_stats_.lock();
  pub_mech_stats_.msg_.controller_statistics.resize(to.size());
  for (size_t i=0; i<to.size(); i++)
    pub_mech_stats_.msg_.controller_statistics[i].name = to[i].name;

  // Destroys the old controllers list when the realtime thread is finished with it.
  ROS_DEBUG("Realtime switches over to new controller list");
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;
  while (ros::ok() && used_by_realtime_ == former_current_controllers_list_){
    if (!ros::ok())
      return false;
    usleep(200);
  }
  ROS_DEBUG("Destruct controller");
  from.clear();
  ROS_DEBUG("Destruct controller finished");
  pub_mech_stats_.unlock();

  ROS_DEBUG("Successfully unloaded controller '%s'", name.c_str());
  return true;
}



bool ControllerManager::switchController(const std::vector<std::string>& start_controllers,
                                         const std::vector<std::string>& stop_controllers,
                                         int strictness)
{
  if (!stop_request_.empty() || !start_request_.empty())
    ROS_FATAL("The switch controller stop and start list are not empty that the beginning of the swithcontroller call. This should not happen.");

  if (strictness == 0){
    ROS_WARN("Controller Manager: To switch controllers you need to specify a strictness level of STRICT or BEST_EFFORT. Defaulting to BEST_EFFORT.");
    strictness = pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
  }

  ROS_DEBUG("switching controllers:");
  for (unsigned int i=0; i<start_controllers.size(); i++)
    ROS_DEBUG(" - starting controller %s", start_controllers[i].c_str());
  for (unsigned int i=0; i<stop_controllers.size(); i++)
    ROS_DEBUG(" - stopping controller %s", stop_controllers[i].c_str());

  // lock controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  pr2_controller_interface::Controller* ct;
  // list all controllers to stop
  for (unsigned int i=0; i<stop_controllers.size(); i++)
  {
    ct = getControllerByName(stop_controllers[i]);
    if (ct == NULL){
      if (strictness ==  pr2_mechanism_msgs::SwitchController::Request::STRICT){
        ROS_ERROR("Could not stop controller with name %s because no controller with this name exists",
                  stop_controllers[i].c_str());
        stop_request_.clear();
        return false;
      }
      else{
        ROS_DEBUG("Could not stop controller with name %s because no controller with this name exists",
                  stop_controllers[i].c_str());
      }
    }
    else{
      ROS_DEBUG("Found controller %s that needs to be stopped in list of controllers",
                stop_controllers[i].c_str());
      stop_request_.push_back(ct);
    }
  }
  ROS_DEBUG("Stop request vector has size %i", (int)stop_request_.size());

  // list all controllers to start
  for (unsigned int i=0; i<start_controllers.size(); i++)
  {
    ct = getControllerByName(start_controllers[i]);
    if (ct == NULL){
      if (strictness ==  pr2_mechanism_msgs::SwitchController::Request::STRICT){
        ROS_ERROR("Could not start controller with name %s because no controller with this name exists",
                  start_controllers[i].c_str());
        stop_request_.clear();
        start_request_.clear();
        return false;
      }
      else{
        ROS_DEBUG("Could not start controller with name %s because no controller with this name exists",
                  start_controllers[i].c_str());
      }
    }
    else{
      ROS_DEBUG("Found controller %s that needs to be started in list of controllers",
                start_controllers[i].c_str());
      start_request_.push_back(ct);
    }
  }
  ROS_DEBUG("Start request vector has size %i", (int)start_request_.size());

  // start the atomic controller switching
  switch_strictness_ = strictness;
  please_switch_ = true;

  // wait until switch is finished
  ROS_DEBUG("Request atomic controller switch from realtime loop");
  while (ros::ok() && please_switch_){
    if (!ros::ok())
      return false;
    usleep(100);
  }
  ROS_DEBUG("Successfully switched controllers");
  return true;
}


void ControllerManager::publishJointState()
{
  ros::Time now = ros::Time::now();
  if (now > last_published_joint_state_ + publish_period_joint_state_)
  {
    if (pub_joint_state_.trylock())
    {
      while (last_published_joint_state_ + publish_period_joint_state_ < now)
        last_published_joint_state_ = last_published_joint_state_ + publish_period_joint_state_;

      unsigned int j = 0;
      for (unsigned int i = 0; i < state_->joint_states_.size(); ++i)
      {
        int type = state_->joint_states_[i].joint_->type;
        if (type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS || type == urdf::Joint::PRISMATIC)
        {
          assert(j < pub_joint_state_.msg_.name.size());
          assert(j < pub_joint_state_.msg_.position.size());
          assert(j < pub_joint_state_.msg_.velocity.size());
          assert(j < pub_joint_state_.msg_.effort.size());
          pr2_mechanism_model::JointState *in = &state_->joint_states_[i];
          pub_joint_state_.msg_.name[j] = state_->joint_states_[i].joint_->name;
          pub_joint_state_.msg_.position[j] = in->position_;
          pub_joint_state_.msg_.velocity[j] = in->velocity_;
          pub_joint_state_.msg_.effort[j] = in->measured_effort_;

          j++;
        }
      }
      pub_joint_state_.msg_.header.stamp = ros::Time::now();
      pub_joint_state_.unlockAndPublish();
    }
  }
}


void ControllerManager::publishMechanismStatistics()
{
  ros::Time now = ros::Time::now();
  if (now > last_published_mechanism_stats_ + publish_period_mechanism_stats_)
  {
    if (pub_mech_stats_.trylock())
    {
      while (last_published_mechanism_stats_ + publish_period_mechanism_stats_ < now)
        last_published_mechanism_stats_ = last_published_mechanism_stats_ + publish_period_mechanism_stats_;

      // joint state
      unsigned int j = 0;
      for (unsigned int i = 0; i < state_->joint_states_.size(); ++i)
      {
        int type = state_->joint_states_[i].joint_->type;
        if (type == urdf::Joint::REVOLUTE || type == urdf::Joint::CONTINUOUS || type == urdf::Joint::PRISMATIC)
        {
          assert(j < pub_mech_stats_.msg_.joint_statistics.size());
          pr2_mechanism_model::JointState *in = &state_->joint_states_[i];
          pr2_mechanism_msgs::JointStatistics *out = &pub_mech_stats_.msg_.joint_statistics[j];
          out->timestamp = now;
          out->name = state_->joint_states_[i].joint_->name;
          out->position = in->position_;
          out->velocity = in->velocity_;
          out->measured_effort = in->measured_effort_;
          out->commanded_effort = in->commanded_effort_;
          out->is_calibrated = in->calibrated_;
          out->violated_limits = in->joint_statistics_.violated_limits_;
          out->odometer = in->joint_statistics_.odometer_;
          out->min_position = in->joint_statistics_.min_position_;
          out->max_position = in->joint_statistics_.max_position_;
          out->max_abs_velocity = in->joint_statistics_.max_abs_velocity_;
          out->max_abs_effort = in->joint_statistics_.max_abs_effort_;
          in->joint_statistics_.reset();
          j++;
        }
      }

      // actuator state
      unsigned int i = 0;
      for (ActuatorMap::const_iterator it = model_.hw_->actuators_.begin(); it != model_.hw_->actuators_.end(); ++i, ++it)
      {
        pr2_mechanism_msgs::ActuatorStatistics *out = &pub_mech_stats_.msg_.actuator_statistics[i];
        ActuatorState *in = &(it->second->state_);
        out->timestamp = now;
        out->name = it->first;
        out->encoder_count = in->encoder_count_;
        out->encoder_offset = in->zero_offset_;
        out->position = in->position_;
        out->timestamp = ros::Time(in->timestamp_);
        out->device_id = in->device_id_;
        out->encoder_velocity = in->encoder_velocity_;
        out->velocity = in->velocity_;
        out->calibration_reading = in->calibration_reading_;
        out->calibration_rising_edge_valid = in->calibration_rising_edge_valid_;
        out->calibration_falling_edge_valid = in->calibration_falling_edge_valid_;
        out->last_calibration_rising_edge = in->last_calibration_rising_edge_;
        out->last_calibration_falling_edge = in->last_calibration_falling_edge_;
        out->is_enabled = in->is_enabled_;
        out->halted = in->halted_;
        out->last_commanded_current = in->last_commanded_current_;
        out->last_executed_current = in->last_executed_current_;
        out->last_measured_current = in->last_measured_current_;
        out->last_commanded_effort = in->last_commanded_effort_;
        out->last_executed_effort = in->last_executed_effort_;
        out->last_measured_effort = in->last_measured_effort_;
        out->motor_voltage = in->motor_voltage_;
        out->num_encoder_errors = in->num_encoder_errors_;
      }

      // controller state
      std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];
      for (unsigned int i = 0; i < controllers.size(); ++i)
      {
        pr2_mechanism_msgs::ControllerStatistics *out = &pub_mech_stats_.msg_.controller_statistics[i];
        out->timestamp = now;
        out->running = controllers[i].c->isRunning();
        out->max_time = ros::Duration(max(controllers[i].stats->acc));
        out->mean_time = ros::Duration(mean(controllers[i].stats->acc));
        out->variance_time = ros::Duration(sqrt(variance(controllers[i].stats->acc)));
        out->num_control_loop_overruns = controllers[i].stats->num_control_loop_overruns;
        out->time_last_control_loop_overrun = controllers[i].stats->time_last_control_loop_overrun;
      }

      pub_mech_stats_.msg_.header.stamp = ros::Time::now();

      pub_mech_stats_.unlockAndPublish();
    }
  }
}



bool ControllerManager::reloadControllerLibrariesSrv(
  pr2_mechanism_msgs::ReloadControllerLibraries::Request &req,
  pr2_mechanism_msgs::ReloadControllerLibraries::Response &resp)
{
  // lock services
  ROS_DEBUG("reload libraries service called");
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("reload libraries service locked");

  // only reload libraries if no controllers are running
  std::vector<std::string> controllers;
  getControllerNames(controllers);
  if (!controllers.empty() && !req.force_kill){
    ROS_ERROR("Controller manager: Cannot reload controller libraries because there are still %i controllers running", (int)controllers.size());
    resp.ok = false;
    return true;
  }

  // kill running controllers if requested
  if (!controllers.empty()){
    ROS_INFO("Controller manager: Killing all running controllers");
    std::vector<std::string> empty;
    if (!switchController(empty,controllers, pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT)){
      ROS_ERROR("Controller manager: Cannot reload controller libraries because failed to stop running controllers");
      resp.ok = false;
      return true;
    }
    for (unsigned int i=0; i<controllers.size(); i++){
      if (!unloadController(controllers[i])){
        ROS_ERROR("Controller manager: Cannot reload controller libraries because failed to unload controller %s",
                  controllers[i].c_str());
        resp.ok = false;
        return true;
      }
    }
    getControllerNames(controllers);
  }
  assert(controllers.empty());

  // create new controller loader
  controller_loader_.reset(new pluginlib::ClassLoader<pr2_controller_interface::Controller>("pr2_controller_interface",
                                                                                            "pr2_controller_interface::Controller"));
  ROS_INFO("Controller manager: reloaded controller libraries");
  resp.ok = true;

  ROS_DEBUG("reload libraries service finished");
  return true;
}


bool ControllerManager::listControllerTypesSrv(
  pr2_mechanism_msgs::ListControllerTypes::Request &req,
  pr2_mechanism_msgs::ListControllerTypes::Response &resp)
{
  // pretend to use the request
  (void) req;

  // lock services
  ROS_DEBUG("list types service called");
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("list types service locked");

  resp.types = controller_loader_->getDeclaredClasses();

  ROS_DEBUG("list types service finished");
  return true;
}


bool ControllerManager::listControllersSrv(
  pr2_mechanism_msgs::ListControllers::Request &req,
  pr2_mechanism_msgs::ListControllers::Response &resp)
{
  // pretend to use the request
  (void) req;

  // lock services
  ROS_DEBUG("list controller service called");
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("list controller service locked");

  std::vector<std::string> controllers;
  std::vector<size_t> schedule;

  getControllerNames(controllers);
  getControllerSchedule(schedule);
  assert(controllers.size() == schedule.size());
  resp.controllers.resize(controllers.size());
  resp.state.resize(controllers.size());

  for (size_t i=0; i<schedule.size(); i++){
    // add controller state
    Controller* c = getControllerByName(controllers[schedule[i]]);
    assert(c);
    resp.controllers[i] = controllers[schedule[i]];
    if (c->isRunning())
      resp.state[i] = "running";
    else
      resp.state[i] = "stopped";
  }

  ROS_DEBUG("list controller service finished");
  return true;
}


bool ControllerManager::loadControllerSrv(
  pr2_mechanism_msgs::LoadController::Request &req,
  pr2_mechanism_msgs::LoadController::Response &resp)
{
  // lock services
  ROS_DEBUG("loading service called for controller %s ",req.name.c_str());
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("loading service locked");

  resp.ok = loadController(req.name);

  ROS_DEBUG("loading service finished for controller %s ",req.name.c_str());
  return true;
}


bool ControllerManager::unloadControllerSrv(
  pr2_mechanism_msgs::UnloadController::Request &req,
  pr2_mechanism_msgs::UnloadController::Response &resp)
{
  // lock services
  ROS_DEBUG("unloading service called for controller %s ",req.name.c_str());
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("unloading service locked");

  resp.ok = unloadController(req.name);

  ROS_DEBUG("unloading service finished for controller %s ",req.name.c_str());
  return true;
}


bool ControllerManager::switchControllerSrv(
  pr2_mechanism_msgs::SwitchController::Request &req,
  pr2_mechanism_msgs::SwitchController::Response &resp)
{
  // lock services
  ROS_DEBUG("switching service called");
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("switching service locked");

  resp.ok = switchController(req.start_controllers, req.stop_controllers, req.strictness);

  ROS_DEBUG("switching service finished");
  return true;
}

