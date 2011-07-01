/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PR2_BELT_COMPENSATOR_TRANSMISSION_H
#define PR2_BELT_COMPENSATOR_TRANSMISSION_H

#include <tinyxml.h>

#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/robot.h>
#include <pr2_mechanism_model/transmission.h>
#include "pr2_mechanism_model/joint_calibration_simulator.h"

namespace pr2_mechanism_model {

/**
 *
 * PROPAGATE*BACKWARDS IS NOT GUARANTEED TO WORK
 */
class PR2BeltCompensatorTransmission : public Transmission
{
public:
  PR2BeltCompensatorTransmission() {}
  ~PR2BeltCompensatorTransmission() {}

  bool initXml(TiXmlElement *config, Robot *robot);
  bool initXml(TiXmlElement *config);

  void propagatePosition(std::vector<pr2_hardware_interface::Actuator*>&,
                         std::vector<pr2_mechanism_model::JointState*>&);
  void propagatePositionBackwards(std::vector<pr2_mechanism_model::JointState*>&,
                                  std::vector<pr2_hardware_interface::Actuator*>&);
  void propagateEffort(std::vector<pr2_mechanism_model::JointState*>&,
                       std::vector<pr2_hardware_interface::Actuator*>&);
  void propagateEffortBackwards(std::vector<pr2_hardware_interface::Actuator*>&,
                                std::vector<pr2_mechanism_model::JointState*>&);

private:
  ros::Duration last_timestamp_;
  double dt;

  // Transmission parameters
  double mechanical_reduction_;
  double trans_compl_;		// Transmission compliance
  double trans_tau_;		// Transmission time constant
  double Kd_motor_;		// Motor damping gain
  double lambda_motor_;		// Damping cutoff bandwidth
  double lambda_joint_;		// Joint estimate rolloff bandwidth
  double lambda_combo_;		// Estimate combination crossover bandwidth

  double last_motor_pos_;
  double last_motor_vel_;

  double last_jnt1_pos_;
  double last_jnt1_vel_;
  double last_jnt1_acc_;

  double last_defl_pos_;
  double last_defl_vel_;
  double last_defl_acc_;

  double last_joint_pos_;
  double last_joint_vel_;

  double delta_motor_vel_;
  double last_motor_damping_force_;


  // Backward transmission states
  ros::Duration last_timestamp_backwards_;
  double halfdt_backwards_;

  double motor_force_backwards_;

  double last_motor_pos_backwards_;
  double last_motor_vel_backwards_;
  double last_motor_acc_backwards_;

  double last_joint_pos_backwards_;
  double last_joint_vel_backwards_;

  int simulated_actuator_timestamp_initialized_;
  ros::Time simulated_actuator_start_time_;


  JointCalibrationSimulator joint_calibration_simulator_;
};

} // namespace

#endif
