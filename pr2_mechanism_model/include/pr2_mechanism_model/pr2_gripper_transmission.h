/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/*
 * <transmission type="PR2GripperTransmission" name="gripper_l_transmission">
 *   <actuator       name="l_gripper_motor" />
 *   <gap_joint      name="l_gripper_joint"              mechanical_reduction="1.0" A="0.05"  B="1.0"  C="0.0" />
 *   <passive_joint  name="l_gripper_l_finger_joint"     />
 *   <passive_joint  name="l_gripper_r_finger_joint"     />
 *   <passive_joint  name="l_gripper_r_finger_tip_joint" />
 *   <passive_joint  name="l_gripper_l_finger_tip_joint" />
 * </transmission>
 *
 * Author: John Hsu
 */

#ifndef GRIPPER_TRANSMISSION_H
#define GRIPPER_TRANSMISSION_H

#include <vector>
#include "tinyxml.h"
#include "pr2_mechanism_model/transmission.h"
#include "pr2_mechanism_model/robot.h"
//#include <fstream>
#include "pr2_mechanism_model/joint_calibration_simulator.h"

namespace pr2_mechanism_model {

class PR2GripperTransmission : public Transmission
{
public:
  PR2GripperTransmission() {use_simulated_actuated_joint_=false;has_simulated_passive_actuated_joint_=false;};
  virtual ~PR2GripperTransmission() {/*myfile.close();*/}

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
  std::string gap_joint_;
  //
  // per Functions Engineering doc, 090224_link_data.xml,
  // here, gap_mechanical_reduction_ transforms FROM ENCODER VALUE TO MOTOR REVOLUTIONS
  //
  double      gap_mechanical_reduction_;

  // if a screw_joint is specified, apply torque based on simulated_reduction_
  double      simulated_reduction_;
  bool        use_simulated_actuated_joint_;
  bool        has_simulated_passive_actuated_joint_;

  // The joint_names_ variable is inherited from Transmission.  In
  // joint_names_, the gap joint is first, followed by all the passive
  // joints.

  // store name for passive joints.  This matches elements 1 to N of joint_names_.
  std::vector<std::string> passive_joints_;

private:
  /// \brief compute gap position, velocity and measured effort from actuator states
  void computeGapStates(double MR,double MR_dot,double MT,
                        double &theta,double &dtheta_dMR,double &dt_dtheta,double &dt_dMR,double &gap_size,double &gap_velocity,double &gap_effort);
  void inverseGapStates(double gap_size,double &MR, double &dMR_dtheta,double &dtheta_dt,double &dMR_dt);
  void inverseGapStates1(double theta,double &MR, double &dMR_dtheta,double &dtheta_dt,double &dMR_dt);

  //
  // SOME CONSTANTS
  // the default theta0 when gap size is 0 is needed to assign passive joint angles
  //
  double screw_reduction_; //   default for alpha2 = 2.0/1000.0; //in meters/revolution
  double gear_ratio_     ; //   default for alpha2 = 29.16; // is a ratio
  double theta0_         ; //   default for alpha2 = 2.97571*M_PI/180.0; // convert to radians
  double phi0_           ; //   default for alpha2 = 29.98717*M_PI/180.0; // convert to radians
  double t0_             ; //   default for alpha2 = 0;  //-0.19543/1000.0; // convert to meters
  double L0_             ; //   default for alpha2 = 34.70821/1000.0; // convert to meters
  double h_              ; //   default for alpha2 = 5.200/1000.0; // convert to meters
  double a_              ; //   default for alpha2 = 67.56801/1000.0; // convert to meters
  double b_              ; //   default for alpha2 = 48.97193/1000.0; // convert to meters
  double r_              ; //   default for alpha2 = 91.50000/1000.0; // convert to meters

#define RAD2MR (1.0/(2.0*M_PI)) // convert radians to motor revolutions
#define TOL 0.00001   // limit for denominators

  int simulated_actuator_timestamp_initialized_;
  ros::Time simulated_actuator_start_time_;

  JointCalibrationSimulator joint_calibration_simulator_;
};

} // namespace pr2_mechanism_model

#endif
