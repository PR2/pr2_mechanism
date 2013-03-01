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
#include "pr2_mechanism_model/simple_transmission.h"

using namespace pr2_mechanism_model;
using namespace pr2_hardware_interface;

PLUGINLIB_EXPORT_CLASS(pr2_mechanism_model::SimpleTransmission,
                         pr2_mechanism_model::Transmission)


bool SimpleTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";

  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel->Attribute("name") : NULL;
  if (!joint_name)
  {
    ROS_ERROR("SimpleTransmission did not specify joint name");
    return false;
  }

  const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
  if (!joint)
  {
    ROS_ERROR("SimpleTransmission could not find joint named \"%s\"", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  Actuator *a;
  if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
  {
    ROS_ERROR("SimpleTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  a->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);

  mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

  // Get screw joint informations
  for (TiXmlElement *j = elt->FirstChildElement("simulated_actuated_joint"); j; j = j->NextSiblingElement("simulated_actuated_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name)
    {
      ROS_ERROR("SimpleTransmission did not specify screw joint name");
      use_simulated_actuated_joint_=false;
    }
    else
    {
      const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
      if (!joint)
      {
        ROS_ERROR("SimpleTransmission could not find screw joint named \"%s\"", joint_name);
        use_simulated_actuated_joint_=false;
      }
      else
      {
        use_simulated_actuated_joint_=true;
        joint_names_.push_back(joint_name);  // The first joint is the gap joint

        // get the thread pitch
        const char *simulated_reduction = j->Attribute("simulated_reduction");
        if (!simulated_reduction)
        {
          ROS_ERROR("SimpleTransmission's joint \"%s\" has no coefficient: simulated_reduction.", joint_name);
          return false;
        }
        try
        {
          simulated_reduction_ = boost::lexical_cast<double>(simulated_reduction);
        }
        catch (boost::bad_lexical_cast &e)
        {
          ROS_ERROR("simulated_reduction (%s) is not a float",simulated_reduction);
          return false;
        }
      }
    }
  }
  return true;
}

bool SimpleTransmission::initXml(TiXmlElement *elt)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";

  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel->Attribute("name") : NULL;
  if (!joint_name)
  {
    ROS_ERROR("SimpleTransmission did not specify joint name");
    return false;
  }
  joint_names_.push_back(joint_name);

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name)
  {
    ROS_ERROR("SimpleTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  actuator_names_.push_back(actuator_name);

  mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

  // Get screw joint informations
  for (TiXmlElement *j = elt->FirstChildElement("simulated_actuated_joint"); j; j = j->NextSiblingElement("simulated_actuated_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name)
    {
      ROS_ERROR("SimpleTransmission screw joint did not specify joint name");
      use_simulated_actuated_joint_=false;
    }
    else
    {
      use_simulated_actuated_joint_=true;
      joint_names_.push_back(joint_name);  // The first joint is the gap joint

      // get the thread pitch
      const char *simulated_reduction = j->Attribute("simulated_reduction");
      if (!simulated_reduction)
      {
        ROS_ERROR("SimpleTransmission's joint \"%s\" has no coefficient: simulated_reduction.", joint_name);
        return false;
      }
      try
      {
        simulated_reduction_ = boost::lexical_cast<double>(simulated_reduction);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("simulated_reduction (%s) is not a float",simulated_reduction);
        return false;
      }
    }
  }
  return true;
}

void SimpleTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 1);
  if (use_simulated_actuated_joint_) {assert(js.size() == 2);}
  else                              {assert(js.size() == 1);}
  js[0]->position_ = (as[0]->state_.position_ / mechanical_reduction_) + js[0]->reference_position_;
  js[0]->velocity_ = as[0]->state_.velocity_ / mechanical_reduction_;
  js[0]->measured_effort_ = as[0]->state_.last_measured_effort_ * mechanical_reduction_;

  if (use_simulated_actuated_joint_)
  {
    // screw joint state is not important to us, fill with zeros
    js[1]->position_ = 0;
    js[1]->velocity_ = 0;
    js[1]->measured_effort_ = 0;
    js[1]->reference_position_ = 0;
    js[1]->calibrated_ = true; // treat passive simulation joints as "calibrated"
  }
}

void SimpleTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 1);
  if (use_simulated_actuated_joint_) {assert(js.size() == 2);}
  else                              {assert(js.size() == 1);}
  as[0]->state_.position_ = (js[0]->position_ - js[0]->reference_position_) * mechanical_reduction_;
  as[0]->state_.velocity_ = js[0]->velocity_ * mechanical_reduction_;
  as[0]->state_.last_measured_effort_ = js[0]->measured_effort_ / mechanical_reduction_;

  // Update the timing (making sure it's initialized).
  if (! simulated_actuator_timestamp_initialized_)
    {
      // Set the time stamp to zero (it is measured relative to the start time).
      as[0]->state_.sample_timestamp_ = ros::Duration(0);

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
    }
  // Set the historical (double) timestamp accordingly.
  as[0]->state_.timestamp_ = as[0]->state_.sample_timestamp_.toSec();

  // simulate calibration sensors by filling out actuator states
  this->joint_calibration_simulator_.simulateJointCalibration(js[0],as[0]);
}

void SimpleTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 1);
  if (use_simulated_actuated_joint_) {assert(js.size() == 2);}
  else                               {assert(js.size() == 1);}
  as[0]->command_.enable_ = true;
  as[0]->command_.effort_ = js[0]->commanded_effort_ / mechanical_reduction_;
}

void SimpleTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 1);
  if (use_simulated_actuated_joint_) {assert(js.size() == 2);}
  else                               {assert(js.size() == 1);}
  if (use_simulated_actuated_joint_)
  {
    // set screw joint effort if simulated
    js[1]->commanded_effort_  = as[0]->command_.effort_ * mechanical_reduction_/simulated_reduction_;
  }
  else
    js[0]->commanded_effort_ = as[0]->command_.effort_ * mechanical_reduction_;
}

