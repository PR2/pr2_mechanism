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

#include <pr2_mechanism_model/pr2_belt_transmission.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(pr2_mechanism_model, PR2BeltCompensatorTransmission,
                        pr2_mechanism_model::PR2BeltCompensatorTransmission,
                        pr2_mechanism_model::Transmission)

namespace pr2_mechanism_model {

bool PR2BeltCompensatorTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
  robot_ = robot;
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";

  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel->Attribute("name") : NULL;
  if (!joint_name)
  {
    ROS_ERROR("PR2BeltCompensatorTransmission did not specify joint name");
    return false;
  }

  const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
  if (!joint)
  {
    ROS_ERROR("PR2BeltCompensatorTransmission could not find joint named \"%s\"", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  pr2_hardware_interface::Actuator *a;
  if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
  {
    ROS_ERROR("PR2BeltCompensatorTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  a->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);

  mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

  TiXmlElement *c = elt->FirstChildElement("compensator");
  if (!c)
  {
    ROS_ERROR("No compensator element given for transmission %s", name_.c_str());
    return false;
  }

  double k_belt;
  const char *k_belt_str = c->Attribute("k_belt");
  if (!k_belt_str) {
    ROS_ERROR("No k_belt given for transmission %s", name_.c_str());
    return false;
  }
  k_belt = atof(k_belt_str);

  double mass_motor;
  const char *mass_motor_str = c->Attribute("mass_motor");
  if (!mass_motor_str) {
    ROS_ERROR("No mass_motor given for transmission %s", name_.c_str());
    return false;
  }
  mass_motor = atof(mass_motor_str);

  trans_compl_ = (k_belt > 0.0 ? 1.0 / k_belt : 0.0);
  trans_tau_ = sqrt(mass_motor * trans_compl_);

  const char *kd_motor_str = c->Attribute("kd_motor");
  if (!kd_motor_str) {
    ROS_ERROR("No kd_motor given for transmission %s", name_.c_str());
    return false;
  }
  Kd_motor_ = atof(kd_motor_str);

  const char *lambda_motor_str = c->Attribute("lambda_motor");
  if (!lambda_motor_str) {
    ROS_ERROR("No lambda_motor given for transmission %s", name_.c_str());
    return false;
  }
  lambda_motor_ = atof(lambda_motor_str);

  const char *lambda_joint_str = c->Attribute("lambda_joint");
  if (!lambda_joint_str) {
    ROS_ERROR("No lambda_joint given for transmission %s", name_.c_str());
    return false;
  }
  lambda_joint_ = atof(lambda_joint_str);

  const char *lambda_combined_str = c->Attribute("lambda_combined");
  if (!lambda_combined_str) {
    ROS_ERROR("No lambda_combined given for transmission %s", name_.c_str());
    return false;
  }
  lambda_combo_ = atof(lambda_combined_str);

  // Initializes the filters
  last_motor_pos_ = last_motor_vel_ = 0;
  last_jnt1_pos_ = last_jnt1_vel_ = last_jnt1_acc_ = 0;
  last_defl_pos_ = last_defl_vel_ = last_defl_acc_ = 0;
  last_joint_pos_ = last_joint_vel_ = 0;
  delta_motor_vel_ = 0;
  last_motor_damping_force_ = 0;

  last_time_ = robot_->getTime();
  return true;
}

void PR2BeltCompensatorTransmission::propagatePosition(
  std::vector<pr2_hardware_interface::Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  ros::Time time = robot_->getTime();
  dt = (time - last_time_).toSec();
  last_time_ = time;

  // These are not the actual "motor" positions.  They are the
  // theoretical joint positions if there were no belt involved.

  // Get the motor position, velocity, measured force.  Note we filter
  // the motor velocity, because we want the least lag for the motor
  // damping feedback.  We will filter the joint velocity.
  double motor_pos = as[0]->state_.position_ / mechanical_reduction_;
  double motor_vel = ( dt>0.0 ? (motor_pos - last_motor_pos_)/dt : 0.0 );

  double motor_measured_force = as[0]->state_.last_measured_effort_ * mechanical_reduction_;


  // Estimate the actual joint position and velocity.  We use two
  // distinct methods (each with their own pros/cons and combine).

  // Method 1: Twice filter the motor position at the resonance mode
  // of the joint against the transmission and locked motor.  This
  // duplicates the mechanical filtering occuring in the
  // transmission.  Obviously, this is an approximation.  In
  // particular the resonance frequency is unknown (we have to assume
  // that lambda_joint_ over-estimates it).  Also, this doesn't
  // account for steady-state forces, i.e. transmission stretch.  So
  // the method is better at higher frequencies.  For numerical
  // stability, upper bound the bandwidth by 2/dt.
  double jnt1_pos, jnt1_vel, jnt1_acc;
  double lam = (dt*lambda_joint_ < 2.0 ? lambda_joint_ : 2.0/dt);

  jnt1_acc = (lam*lam * motor_pos
              - lam*lam * (last_jnt1_pos_ + dt*last_jnt1_vel_ + 0.25*dt*dt*last_jnt1_acc_)
              - 2.0*lam * (last_jnt1_vel_ + 0.5*dt*last_jnt1_acc_))
    / (1.0 + 0.5*dt*2.0*lam + 0.25*dt*dt*lam*lam);

  jnt1_vel = last_jnt1_vel_ + 0.5*dt*(jnt1_acc + last_jnt1_acc_);
  jnt1_pos = last_jnt1_pos_ + 0.5*dt*(jnt1_vel + last_jnt1_vel_);

  // Method 2: Estimate the transmission deflection explicitly.  This
  // uses only the transmission stiffness (compliance) and motor mass.
  // The later is combined with the compliance to give a transmission
  // time constant (tau).  This method assumes the motor mass it is
  // much smaller than the joint mass and the resonance of the motor
  // against the transmission is damped.  It does NOT need to know the
  // joint/link mass (i.e. is independent of joint configurations) and
  // adds the appropriate DC transmission stretch.  However, is uses
  // the encoder directly at high frequency, remaining noisy.  For
  // numerical stability, if the time constant is zero, implement a
  // 0th order system.  Else lower bound the time constant by dt/2.
  double defl_pos, defl_vel, defl_acc;
  if (trans_tau_ == 0)
  {
    defl_acc = 0.0;
    defl_vel = 0.0;
    defl_pos = trans_compl_ * motor_measured_force;
  }
  else
  {
    double tau = (dt < 2.0*trans_tau_ ? trans_tau_ : dt/2.0);

    defl_acc = (trans_compl_ * motor_measured_force
                -           (last_defl_pos_ + dt*last_defl_vel_ + 0.25*dt*dt*last_defl_acc_)
                - 2.0*tau * (last_defl_vel_ + 0.5*dt*last_defl_acc_))
      / (tau*tau + 2.0*tau*0.5*dt + 0.25*dt*dt);

    defl_vel = last_defl_vel_ + 0.5*dt*(defl_acc + last_defl_acc_);
    defl_pos = last_defl_pos_ + 0.5*dt*(defl_vel + last_defl_vel_);
  }

  double jnt2_pos = motor_pos - defl_pos;

  // Combine the two joint position estimates and calculate the
  // velocity: High pass method 1, low pass method 2.  In the end, the
  // encoder and measured force are both filtered at least once before
  // reaching the joint estimate.  I.e. the velocity is smooth.  For
  // numerical stability, upper bound the combination bandwidth.
  lam = (dt*lambda_combo_ < 2.0 ? lambda_combo_ : 2.0/dt);

  double joint_pos, joint_vel;
  joint_vel = (jnt1_vel
               + lam * jnt2_pos
               - lam * (last_joint_pos_ + 0.5*dt*last_joint_vel_))
    / (1.0 + 0.5*dt*lam);
  joint_pos = last_joint_pos_ + 0.5*dt*(joint_vel + last_joint_vel_);

  js[0]->position_ = joint_pos + js[0]->reference_position_;
  js[0]->velocity_ = joint_vel;
  js[0]->measured_effort_ = as[0]->state_.last_measured_effort_ * mechanical_reduction_;


  // Stores values used by propogateEffort
  delta_motor_vel_ = motor_vel - last_motor_vel_;

  // Saves the current values for use in the next servo cycle.  These
  // are used to filter the signals
  last_motor_pos_   = motor_pos;
  last_motor_vel_   = motor_vel;

  last_jnt1_pos_    = jnt1_pos;
  last_jnt1_vel_    = jnt1_vel;
  last_jnt1_acc_    = jnt1_acc;

  last_defl_pos_    = defl_pos;
  last_defl_vel_    = defl_vel;
  last_defl_acc_    = defl_acc;

  last_joint_pos_   = joint_pos;
  last_joint_vel_   = joint_vel;
}

void PR2BeltCompensatorTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<pr2_hardware_interface::Actuator*>& as)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  as[0]->state_.position_ = (js[0]->position_ - js[0]->reference_position_) * mechanical_reduction_;
  as[0]->state_.velocity_ = js[0]->velocity_ * mechanical_reduction_;
  as[0]->state_.last_measured_effort_ = js[0]->measured_effort_ / mechanical_reduction_;
}

void PR2BeltCompensatorTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<pr2_hardware_interface::Actuator*>& as)
{
  assert(as.size() == 1);
  assert(js.size() == 1);


  double lam = (dt*lambda_motor_ < 2.0 ? lambda_motor_ : 2.0/dt);

  double motor_damping_force = ((1.0-0.5*dt*lam) * last_motor_damping_force_
                                - Kd_motor_ * delta_motor_vel_)
    / (1.0+0.5*dt*lam);

  // Add to the joint force.
  double motor_force = js[0]->commanded_effort_ + motor_damping_force;

  // Send out!
  as[0]->command_.effort_ = motor_force / mechanical_reduction_;

  last_motor_damping_force_ = motor_damping_force;
}

void PR2BeltCompensatorTransmission::propagateEffortBackwards(
  std::vector<pr2_hardware_interface::Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  js[0]->commanded_effort_ = as[0]->command_.effort_ * mechanical_reduction_;
}

} // namespace
