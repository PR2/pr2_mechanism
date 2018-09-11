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
/*
 * Author: Melonee Wise
 */
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include "pr2_mechanism_model/robot.h"
#include "pr2_mechanism_model/wrist_transmission.h"

using namespace pr2_mechanism_model;
using namespace pr2_hardware_interface;

PLUGINLIB_EXPORT_CLASS(pr2_mechanism_model::WristTransmission,
                        pr2_mechanism_model::Transmission)


static bool convertDouble(const char* val_str, double &value)
{
  char *endptr=NULL;
  value = strtod(val_str, &endptr);
  if ((endptr == val_str) || (endptr < (val_str+strlen(val_str))))
  {
    return false;
  }

  return true;
}

WristTransmission::WristTransmission()
{
  joint_offset_[0] = 0.0;
  joint_offset_[1] = 0.0;
}


bool WristTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";


  TiXmlElement *ael = elt->FirstChildElement("rightActuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  Actuator *a;
  if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
  {
    ROS_WARN("WristTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  a->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);
  const char *act_red = ael->Attribute("mechanicalReduction");
  if (!act_red)
    {
      ROS_WARN("WristTransmission's actuator \"%s\" was not given a reduction.", actuator_name);
      return false;
    }
  actuator_reduction_.push_back(atof(act_red));

  ael = elt->FirstChildElement("leftActuator");
  actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
  {
    ROS_WARN("WristTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  a->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);
  act_red = ael->Attribute("mechanicalReduction");
  if (!act_red)
    {
      ROS_WARN("WristTransmission's actuator \"%s\" was not given a reduction.", actuator_name);
      return false;
    }
  actuator_reduction_.push_back(atof(act_red));


  TiXmlElement *j = elt->FirstChildElement("flexJoint");
  const char *joint_name = j->Attribute("name");
  if (!joint_name)
  {
    ROS_ERROR("WristTransmission did not specify joint name");
    return false;
  }
#if URDFDOM_1_0_0_API
  urdf::JointConstSharedPtr joint = robot->robot_model_.getJoint(joint_name);
#else
  const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
#endif

  if (!joint)
  {
    ROS_ERROR("WristTransmission could not find joint named \"%s\"", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);
  const char *joint_red = j->Attribute("mechanicalReduction");
  if (!joint_red)
  {
    ROS_WARN("WristTransmission's joint \"%s\" was not given a reduction.", joint_name);
    return false;
  }
  joint_reduction_.push_back(atof(joint_red));
  const char *joint_offset = j->Attribute("offset");
  if (!joint_offset)
  {
    joint_offset_[0] = 0.0;
  }
  else
  {
    if (!convertDouble(joint_offset, joint_offset_[0]))
    {
      ROS_WARN("WristTransmission's joint \"%s\", cannot convert jointOffset attribute \"%s\" to floating point.",
               joint_name, joint_offset);
      return false;
    }
    else
    {
      ROS_WARN("Joint offset of %f for joint %s.", joint_offset_[0], joint_name);
    }
  }

  j = elt->FirstChildElement("rollJoint");
  joint_name = j->Attribute("name");
  if (!joint_name)
  {
    ROS_ERROR("WristTransmission did not specify joint name");
    return false;
  }
#if URDFDOM_1_0_0_API
  urdf::JointConstSharedPtr joint2 = robot->robot_model_.getJoint(joint_name);
#else
  const boost::shared_ptr<const urdf::Joint> joint2 = robot->robot_model_.getJoint(joint_name);
#endif

  if (!joint2)
  {
    ROS_ERROR("WristTransmission could not find joint named \"%s\"", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);
  joint_red = j->Attribute("mechanicalReduction");
  if (!joint_red)
  {
    ROS_WARN("WristTransmission's joint \"%s\" was not given a reduction.", joint_name);
    return false;
  }
  joint_reduction_.push_back(atof(joint_red));
  joint_offset = j->Attribute("offset");
  if (!joint_offset)
  {
    joint_offset_[1] = 0.0;
  }
  else
  {
    if (!convertDouble(joint_offset, joint_offset_[1]))
    {
      ROS_WARN("WristTransmission's joint \"%s\", cannot convert jointOffset attribute \"%s\" to floating point.",
               joint_name, joint_offset);
      return false;
    }
    else
    {
      ROS_WARN("Joint offset of %f for joint %s.", joint_offset_[1], joint_name); 
    }
  }


  return true;
}

bool WristTransmission::initXml(TiXmlElement *elt)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";


  TiXmlElement *ael = elt->FirstChildElement("rightActuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name)
  {
    ROS_WARN("WristTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  actuator_names_.push_back(actuator_name);
  const char *act_red = ael->Attribute("mechanicalReduction");
  if (!act_red)
  {
    ROS_WARN("WristTransmission's actuator \"%s\" was not given a reduction.", actuator_name);
    return false;
  }
  actuator_reduction_.push_back(atof(act_red));

  ael = elt->FirstChildElement("leftActuator");
  actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name)
  {
    ROS_WARN("WristTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  actuator_names_.push_back(actuator_name);
  act_red = ael->Attribute("mechanicalReduction");
  if (!act_red)
  {
    ROS_WARN("WristTransmission's actuator \"%s\" was not given a reduction.", actuator_name);
    return false;
  }
  actuator_reduction_.push_back(atof(act_red));


  TiXmlElement *j = elt->FirstChildElement("flexJoint");
  const char *joint_name = j->Attribute("name");
  if (!joint_name)
  {
    ROS_ERROR("WristTransmission did not specify joint name");
    return false;
  }
  joint_names_.push_back(joint_name);
  const char *joint_red = j->Attribute("mechanicalReduction");
  if (!joint_red)
  {
    ROS_WARN("WristTransmission's joint \"%s\" was not given a reduction.", joint_name);
    return false;
  }
  joint_reduction_.push_back(atof(joint_red));
  const char *joint_offset = j->Attribute("offset");
  if (!joint_offset)
  {
    joint_offset_[0] = 0.0;
  }
  else
  {
    double offset;
    if (!convertDouble(joint_offset, joint_offset_[0]))
    {
      ROS_WARN("WristTransmission's joint \"%s\", cannot convert jointOffset attribute \"%s\" to floating point.",
               joint_name, joint_offset);
      return false;
    }
    else
    {
      ROS_WARN("Joint offset of %f for joint %s.", joint_offset_[0], joint_name);
    }
  }

  j = elt->FirstChildElement("rollJoint");
  joint_name = j->Attribute("name");
  if (!joint_name)
  {
    ROS_ERROR("WristTransmission did not specify joint name");
    return false;
  }
  joint_names_.push_back(joint_name);
  joint_red = j->Attribute("mechanicalReduction");
  if (!joint_red)
  {
    ROS_WARN("WristTransmission's joint \"%s\" was not given a reduction.", joint_name);
    return false;
  }
  joint_reduction_.push_back(atof(joint_red));
  if (!joint_offset)
  {
    joint_offset_[1] = 0.0;
  }
  else
  {
    if (!convertDouble(joint_offset, joint_offset_[1]))
    {
      ROS_WARN("WristTransmission's joint \"%s\", cannot convert jointOffset attribute \"%s\" to floating point.",
               joint_name, joint_offset);
      return false;
    }
    else
    {
      ROS_WARN("Joint offset of %f for joint %s.", joint_offset_[1], joint_name);
    }
  }

  return true;
}

void WristTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 2);
  assert(js.size() == 2);

  js[0]->position_ = ((as[0]->state_.position_ / actuator_reduction_[0] - as[1]->state_.position_ / actuator_reduction_[1])/
		      (2*joint_reduction_[0])) + js[0]->reference_position_+joint_offset_[0];
  js[0]->velocity_ = (as[0]->state_.velocity_ / actuator_reduction_[0] - as[1]->state_.velocity_ / actuator_reduction_[1])/(2*joint_reduction_[0]);
  js[0]->measured_effort_ = joint_reduction_[0]*(as[0]->state_.last_measured_effort_ * actuator_reduction_[0] - as[1]->state_.last_measured_effort_ * actuator_reduction_[1]);

  js[1]->position_ = ((-as[0]->state_.position_ / actuator_reduction_[0] - as[1]->state_.position_ / actuator_reduction_[1])/
		      (2*joint_reduction_[1]))+js[1]->reference_position_+joint_offset_[1];
  js[1]->velocity_ = (-as[0]->state_.velocity_ / actuator_reduction_[0] - as[1]->state_.velocity_ / actuator_reduction_[1])/(2*joint_reduction_[1]);
  js[1]->measured_effort_ = joint_reduction_[1]*(-as[0]->state_.last_measured_effort_ * actuator_reduction_[0] - as[1]->state_.last_measured_effort_ * actuator_reduction_[1]);
}

void WristTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 2);
  assert(js.size() == 2);

  as[0]->state_.position_ = (((js[0]->position_-js[0]->reference_position_-joint_offset_[0])*joint_reduction_[0] -
			      (js[1]->position_-js[1]->reference_position_-joint_offset_[1])*joint_reduction_[1]) * actuator_reduction_[0]);
  as[0]->state_.velocity_ = ((js[0]->velocity_*joint_reduction_[0] - js[1]->velocity_*joint_reduction_[1]) * actuator_reduction_[0]);
  as[0]->state_.last_measured_effort_ = (js[0]->measured_effort_/joint_reduction_[0] - js[1]->measured_effort_/joint_reduction_[1]) /(2.0*actuator_reduction_[0]);

  as[1]->state_.position_ = ((-(js[0]->position_-js[0]->reference_position_-joint_offset_[0])*joint_reduction_[0] -
			       (js[1]->position_-js[1]->reference_position_-joint_offset_[1])*joint_reduction_[1]) * actuator_reduction_[1]);
  as[1]->state_.velocity_ = ((-js[0]->velocity_*joint_reduction_[0] - js[1]->velocity_*joint_reduction_[1]) * actuator_reduction_[1]);
  as[1]->state_.last_measured_effort_ = (-js[0]->measured_effort_/joint_reduction_[0] - js[1]->measured_effort_/joint_reduction_[1]) /(2.0*actuator_reduction_[1]);

  // Update the timing (making sure it's initialized).
  if (! simulated_actuator_timestamp_initialized_)
    {
      // Set the time stamp to zero (it is measured relative to the start time).
      as[0]->state_.sample_timestamp_ = ros::Duration(0);
      as[1]->state_.sample_timestamp_ = ros::Duration(0);

      // Try to set the start time.  Only then do we claim initialized.
      if (ros::isStarted())
	{
	  simulated_actuator_start_time_ = ros::Time::now();
	  simulated_actuator_timestamp_initialized_ = true;
	}
    }
  else
    {
      // Measure the time stamp relative to the start time.
      as[0]->state_.sample_timestamp_ = ros::Time::now() - simulated_actuator_start_time_;
      as[1]->state_.sample_timestamp_ = ros::Time::now() - simulated_actuator_start_time_;
    }
  // Set the historical (double) timestamp accordingly.
  as[0]->state_.timestamp_ = as[0]->state_.sample_timestamp_.toSec();
  as[1]->state_.timestamp_ = as[1]->state_.sample_timestamp_.toSec();

  // simulate calibration sensors by filling out actuator states
  // this is where to embed the hack which joint connects to which mcb
  this->joint_calibration_simulator_[0].simulateJointCalibration(js[0],as[1]);
  this->joint_calibration_simulator_[1].simulateJointCalibration(js[1],as[0]);
}

void WristTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 2);
  assert(js.size() == 2);

  as[0]->command_.enable_ = true;
  as[1]->command_.enable_ = true;

  as[0]->command_.effort_ = (js[0]->commanded_effort_/joint_reduction_[0] - js[1]->commanded_effort_/joint_reduction_[1]) /(2.0*actuator_reduction_[0]);
  as[1]->command_.effort_ = (-js[0]->commanded_effort_/joint_reduction_[0] - js[1]->commanded_effort_/joint_reduction_[1]) /(2.0*actuator_reduction_[1]);
}

void WristTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 2);
  assert(js.size() == 2);

  js[0]->commanded_effort_ = joint_reduction_[0]*(as[0]->command_.effort_ * actuator_reduction_[0] - as[1]->command_.effort_ * actuator_reduction_[1]);
  js[1]->commanded_effort_ = joint_reduction_[1]*(-as[0]->command_.effort_ * actuator_reduction_[0] - as[1]->command_.effort_ * actuator_reduction_[1]);
}


void WristTransmission::setReductions(std::vector<double>& ar, std::vector<double>& jr)
{
  actuator_reduction_ = ar;
  joint_reduction_ = jr;
}

