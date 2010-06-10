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
#ifndef SIMPLE_TRANSMISSION_H
#define SIMPLE_TRANSMISSION_H

#include <ros/ros.h>
#include <tinyxml/tinyxml.h>
#include "pr2_mechanism_model/transmission.h"
#include "pr2_mechanism_model/joint.h"
#include "pr2_hardware_interface/hardware_interface.h"
#include <std_msgs/Float64.h>
#include <filters/filter_chain.h>

namespace pr2_mechanism_model {

class SimpleOffsetTransmission : public Transmission
{
public:
 SimpleOffsetTransmission() : offset_filter_("double") {}
  ~SimpleOffsetTransmission() {}

  bool initXml(TiXmlElement *config, Robot *robot);

  ros::NodeHandle nh_;
  ros::Subscriber offset_sub_;
  double offset_;
  double filtered_offset_;

  double mechanical_reduction_;

  void offsetCallback(const std_msgs::Float64ConstPtr& offset);

  void propagatePosition(std::vector<pr2_hardware_interface::Actuator*>&, 
                         std::vector<pr2_mechanism_model::JointState*>&);
  void propagatePositionBackwards(std::vector<pr2_mechanism_model::JointState*>&, 
                                  std::vector<pr2_hardware_interface::Actuator*>&);
  void propagateEffort(std::vector<pr2_mechanism_model::JointState*>&, 
                       std::vector<pr2_hardware_interface::Actuator*>&);
  void propagateEffortBackwards(std::vector<pr2_hardware_interface::Actuator*>&, 
                                std::vector<pr2_mechanism_model::JointState*>&);

  filters::FilterChain<double> offset_filter_;
};

} // namespace pr2_mechanism_model

#endif
