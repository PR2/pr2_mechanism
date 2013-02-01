/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
#pragma once
/***************************************************/
/*! \namespace controller
 \brief The controller namespace

 \class controller::Controller
 \brief A base level controller class.

 */
/***************************************************/

#include <ros/node_handle.h>
#include <pr2_mechanism_model/robot.h>
#include <pr2_controller_interface/controller_provider.h>
#include <controller_interface/controller.h>

namespace pr2_controller_interface
{

class Controller : public controller_interface::Controller<pr2_mechanism_model::RobotState >
{
public:
  enum {BEFORE_ME, AFTER_ME};

  Controller(): state_(CONSTRUCTED){}
  virtual ~Controller(){}

  void starting(const ros::Time& time) { starting(); }
  void update  (const ros::Time& time, const ros::Duration& period) { update();   }
  void stopping(const ros::Time& time) { stopping(); }

  /// The starting method is called just before the first update from within the realtime thread.
  virtual void starting() {};

  /// The update method is called periodically by the realtime thread when the controller is running
  virtual void update(void) = 0;

  /// The stopping method is called by the realtime thread just after the last update call
  virtual void stopping() {};

  /**
   * @brief The init function is called to initialize the controller from a non-realtime thread
   *
   * @param robot A RobotState object which can be used to read joint
   * states and write out effort commands.
   *
   * @param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * @return True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n) = 0;

  /// Method to get access to another controller by name and type.
  template<class ControllerType> bool getController(const std::string& name, int sched, ControllerType*& c)
  {
    if (contr_prov_ == NULL){
      ROS_ERROR("No valid pointer to a controller provider exists");
      return false;
    }
    if (!contr_prov_->getControllerByName(name, c)){
      ROS_ERROR("Could not find controller %s", name.c_str());
      return false;
    }
    if (sched == BEFORE_ME) before_list_.push_back(name);
    else if (sched == AFTER_ME) after_list_.push_back(name);
    else{
      ROS_ERROR("No valid scheduling specified. Need BEFORE_ME or AFTER_ME in getController function");
      return false;
    }
    return true;
  }

  /// Check if the controller is running
  bool isRunning()
  {
    return (state_ == RUNNING);
  }

  void updateRequest()
  {
    if (state_ == RUNNING)
      update();
  }

  bool startRequest()
  {
    // start succeeds even if the controller was already started
    if (state_ == RUNNING || state_ == INITIALIZED){
      starting();
      state_ = RUNNING;
      return true;
    }
    else
      return false;
  }


  bool stopRequest()
  {
    // stop succeeds even if the controller was already stopped
    if (state_ == RUNNING || state_ == INITIALIZED){
      stopping();
      state_ = INITIALIZED;
      return true;
    }
    else
      return false;
  }

  bool initRequest(ControllerProvider* cp, pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    contr_prov_ = cp;

    if (state_ != CONSTRUCTED)
      return false;
    else
    {
      // initialize
      if (!init(robot, n))
        return false;
      state_ = INITIALIZED;

      return true;
    }
  }


  std::vector<std::string> before_list_, after_list_;

  enum {CONSTRUCTED, INITIALIZED, RUNNING} state_;

private:
  Controller(const Controller &c);
  Controller& operator =(const Controller &c);
  ControllerProvider* contr_prov_;

};

}
