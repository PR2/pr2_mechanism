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
 * Author: Stuart Glaser
 */
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include "pr2_mechanism_model/robot.h"
#include "pr2_mechanism_model/simple_offset_transmission.h"

using namespace pr2_mechanism_model;
using namespace pr2_hardware_interface;

PLUGINLIB_REGISTER_CLASS(SimpleOffsetTransmission, 
                         pr2_mechanism_model::SimpleOffsetTransmission, 
                         pr2_mechanism_model::Transmission)


bool SimpleOffsetTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";

  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel->Attribute("name") : NULL;
  if (!joint_name)
  {
    ROS_ERROR("SimpleOffsetTransmission did not specify joint name");
    return false;
  }

  const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
  if (!joint)
  {
    ROS_ERROR("SimpleOffsetTransmission could not find joint named \"%s\"", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  Actuator *a;
  if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
  {
    ROS_ERROR("SimpleOffsetTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  a->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);

  mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

  offset_ = 0.0;
  std::string offset_topic = std::string(joint_name) + "/offset";
  offset_sub_ = nh_.subscribe(offset_topic, 1, &SimpleOffsetTransmission::offsetCallback, this);

  if (!offset_filter_.configure("/servo_filter_chain", nh_))
  {
    ROS_ERROR("OffsetTransmission: Error Constructing Filter Chain");
    return false;
  }
  return true;
}

void SimpleOffsetTransmission::offsetCallback(const std_msgs::Float64ConstPtr& offset)
{
  offset_ = offset->data;
}

void SimpleOffsetTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  double filtered_offset;
  double offset = offset_;
  offset_filter_.update(offset, filtered_offset);
  filtered_offset_ = filtered_offset;

  // Make offset decay slowly
  offset_ = offset * 0.984;

  assert(as.size() == 1);
  assert(js.size() == 1);
  js[0]->position_ = as[0]->state_.position_ / mechanical_reduction_ - filtered_offset_;
  //js[0]->position_ = as[0]->state_.position_ / mechanical_reduction_;
  js[0]->velocity_ = as[0]->state_.velocity_ / mechanical_reduction_;
  js[0]->measured_effort_ = as[0]->state_.last_measured_effort_ * mechanical_reduction_;
}

void SimpleOffsetTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  as[0]->state_.position_ = (js[0]->position_ + filtered_offset_) * mechanical_reduction_;
  //as[0]->state_.position_ = js[0]->position_ * mechanical_reduction_;
  as[0]->state_.velocity_ = js[0]->velocity_ * mechanical_reduction_;
  as[0]->state_.last_measured_effort_ = js[0]->measured_effort_ / mechanical_reduction_;
}

void SimpleOffsetTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  as[0]->command_.effort_ = js[0]->commanded_effort_ / mechanical_reduction_;
}

void SimpleOffsetTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  js[0]->commanded_effort_ = as[0]->command_.effort_ * mechanical_reduction_;
}
