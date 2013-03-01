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

PLUGINLIB_EXPORT_CLASS(pr2_mechanism_model::PR2BeltCompensatorTransmission,
                        pr2_mechanism_model::Transmission)

namespace pr2_mechanism_model {

bool PR2BeltCompensatorTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
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

  last_timestamp_ = ros::Duration(0);

  // Initialize the backward transmission state variables.
  last_timestamp_backwards_ = last_timestamp_;
  halfdt_backwards_ = 0.0;
  motor_force_backwards_ = 0.0;
  last_motor_pos_backwards_ = 0.0;
  last_motor_vel_backwards_ = 0.0;
  last_motor_acc_backwards_ = 0.0;
  last_joint_pos_backwards_ = 0.0;
  last_joint_vel_backwards_ = 0.0;

  return true;
}

bool PR2BeltCompensatorTransmission::initXml(TiXmlElement *elt)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";

  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel->Attribute("name") : NULL;
  if (!joint_name)
  {
    ROS_ERROR("PR2BeltCompensatorTransmission did not specify joint name");
    return false;
  }

  joint_names_.push_back(joint_name);

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
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

  last_timestamp_ = ros::Duration(0);

  // Initialize the backward transmission state variables.
  last_timestamp_backwards_ = last_timestamp_;
  halfdt_backwards_ = 0.0;
  motor_force_backwards_ = 0.0;
  last_motor_pos_backwards_ = 0.0;
  last_motor_vel_backwards_ = 0.0;
  last_motor_acc_backwards_ = 0.0;
  last_joint_pos_backwards_ = 0.0;
  last_joint_vel_backwards_ = 0.0;

  return true;
}

void PR2BeltCompensatorTransmission::propagatePosition(
  std::vector<pr2_hardware_interface::Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  ros::Duration timestamp = as[0]->state_.sample_timestamp_;
  dt = (timestamp - last_timestamp_).toSec();
  last_timestamp_ = timestamp;

  // These are not the actual "motor" positions.  They are the
  // theoretical joint positions if there were no belt involved.

  // Get the motor position, velocity, measured force.  Note we do not
  // filter the motor velocity, because we want the least lag for the
  // motor damping feedback.  We will filter the joint velocity.
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

  jnt1_vel = last_jnt1_vel_ + 0.5*dt*(           last_jnt1_acc_);
  jnt1_pos = last_jnt1_pos_ + 0.5*dt*(jnt1_vel + last_jnt1_vel_);

  jnt1_acc = (lam*lam * (motor_pos-jnt1_pos) - 2.0*lam * jnt1_vel)
           / (1.0 + 0.5*dt*2.0*lam + 0.25*dt*dt*lam*lam);

  jnt1_vel = last_jnt1_vel_ + 0.5*dt*(jnt1_acc + last_jnt1_acc_);
  jnt1_pos = last_jnt1_pos_ + 0.5*dt*(jnt1_vel + last_jnt1_vel_);

  // Method 2: Estimate the transmission deflection explicitly.  This
  // uses only the transmission stiffness (compliance) and motor mass.
  // The later is combined with the compliance to give a transmission
  // time constant (tau).  This method assumes the motor mass is much
  // smaller than the joint mass and the resonance of the motor
  // against the transmission is damped.  It does NOT need to know the
  // joint/link mass (i.e. is independent of joint configurations) and
  // adds the appropriate DC transmission stretch.  However, it
  // assumes zero motor friction to ground and no Coulomb fiction.
  // Also, it uses the encoder directly at high frequency, remaining
  // noisy.  For numerical stability, if the time constant is zero,
  // implement a 0th order system.  Else lower bound the time constant
  // by dt/2.
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

    defl_vel = last_defl_vel_ + 0.5*dt*(           last_defl_acc_);
    defl_pos = last_defl_pos_ + 0.5*dt*(defl_vel + last_defl_vel_);

    defl_acc = (trans_compl_ * motor_measured_force
                - defl_pos - 2.0*tau * defl_vel)
             / (tau*tau + 2.0*tau*0.5*dt + 0.25*dt*dt);

    defl_vel = last_defl_vel_ + 0.5*dt*(defl_acc + last_defl_acc_);
    defl_pos = last_defl_pos_ + 0.5*dt*(defl_vel + last_defl_vel_);
  }

  double jnt2_pos = motor_pos - defl_pos;

  // Combine the two joint position estimates and calculate the
  // velocity: High pass method 1, low pass method 2.  In the end, the
  // encoder and measured force are both filtered at least once before
  // reaching the joint estimate.  I.e. the velocity is smooth.  For
  // numerical stability, upper bound the combination bandwidth.  If
  // the bandwidth is zero, take just method 1.
  double joint_pos, joint_vel;
  if (lambda_combo_ == 0.0)
  {
    joint_pos = jnt1_pos;
    joint_vel = jnt1_vel;
  }
  else
  {
    lam = (dt*lambda_combo_ < 2.0 ? lambda_combo_ : 2.0/dt);

    joint_pos = last_joint_pos_ + 0.5*dt*(last_joint_vel_);
    joint_vel = (jnt1_vel + lam * (jnt2_pos - joint_pos))
              / (1.0 + 0.5*dt*lam);
    joint_pos = last_joint_pos_ + 0.5*dt*(joint_vel + last_joint_vel_);
  }


  // Push the joint info out.
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

void PR2BeltCompensatorTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<pr2_hardware_interface::Actuator*>& as)
{
  assert(as.size() == 1);
  assert(js.size() == 1);


  // Calculate the damping force for the motor vibrations against the
  // transmission.  As we don't have a true vibration measurement,
  // dampen the motor simply at high frequency.  For numerical
  // stability, upper bound the cutoff bandwidth.  If the bandwidth is
  // zero, use damping over all frequencies, i.e. regular damping.
  double motor_damping_force;
  if (lambda_motor_ == 0.0)
  {
    motor_damping_force = - Kd_motor_ * last_motor_vel_;
  }
  else
  {
    double lam = (dt*lambda_motor_ < 2.0 ? lambda_motor_ : 2.0/dt);

    motor_damping_force = ((1.0-0.5*dt*lam) * last_motor_damping_force_
                           - Kd_motor_ * delta_motor_vel_)
                        / (1.0+0.5*dt*lam);
  }

  // Add to the joint force.
  double motor_force = js[0]->commanded_effort_ + motor_damping_force;

  // Send out!
  as[0]->command_.effort_ = motor_force / mechanical_reduction_;

  last_motor_damping_force_ = motor_damping_force;
}



// The backward transmission is a entirely separate entity for the
// forward transmission.  Both contain state information, that is NOT
// shared.  In particular, the backward transmission implements a
// 4th-order model of the motor mass, belt stiffness, and joint mass,
// which is what the forward transmission expects.  It should be used
// ONLY in Gazebo, i.e. when simulating the robot.  As such, we assume
// here that a cycle starts with a given actuator effort,
// propagateEffortBackwards() is called first (setting the time step) to
// set the joint effort, Gazebo then calculates a new joint position,
// and propagatePositionBackwards() is called last to provide the new
// actuator position.
void PR2BeltCompensatorTransmission::propagateEffortBackwards(
  std::vector<pr2_hardware_interface::Actuator*>& as, std::vector<JointState*>& js)
{
  // Check the arguments.  This acts only on a single actuator/joint.
  assert(as.size() == 1);
  assert(js.size() == 1);

  // We are simulating a motor-mass / belt-spring-damper / joint-mass
  // system.  Note all calculations are done if joint-space units, so
  // that the motor position/velocity/acceleration/force are scaled
  // appropriately by the mechanical reduction.  Also note, the joint
  // mass is actually simulated in Gazebo, so for the purposes of this
  // belt simulation, we assume an infinite joint mass and hence zero
  // joint acceleration.  Finally, we assume the belt-spring/motor-mass
  // dynamics are critically damped and set the damping accordingly.
  double motor_pos, motor_vel, motor_acc;
  double joint_pos, joint_vel;
  double motor_force;
  double spring_force;
  double halfdt;

  // Calculate the time step.  Should be the same as the forward
  // transmission, but we don't want/need to assume that.  Furthermore,
  // given this is used only to simulate a transmission, the time-steps
  // should be perfectly constant and known in advance.  But to keep
  // this clean we recalculate.  The question remains who defines the
  // time step.  Like the forward transmission, we can only use the time
  // step in the actuator state, though this makes little sense here...
  ros::Duration timestamp = as[0]->state_.sample_timestamp_;
  halfdt = 0.5*(timestamp - last_timestamp_backwards_).toSec();
  last_timestamp_backwards_ = timestamp;

  // Get the actuator force acting on the motor mass, multipled by the
  // mechanical reduction to be in joint-space units.  Note we are
  // assuming the command is perfectly executed, i.e. will completely
  // act on the motor.
  motor_force = as[0]->command_.effort_ * mechanical_reduction_;

  // If the transmission compliance is zero (infinitely stiff) or the
  // motor mass is zero (infinitely light), then the transmission time
  // constant is also zero (infinitely fast) and the entire transmission
  // collapses to a regular rigid connection.
  if ((trans_compl_ == 0.0) || (trans_tau_ == 0.0))
    {
      // Immediately propagate the motor force to the spring, ignoring
      // the (infinitely fast) model dynamics.
      spring_force = motor_force;
    }
  else
    {
      // Update the model.  Note for numerical stability, the
      // transmission dynamics need to be slower than the integration
      // time step.  Specifically we need to lower bound the tranmission
      // time constant by dt/2.
      double tau = (halfdt < trans_tau_ ? trans_tau_ : halfdt);

      // Calculate the new motor position/velocity assuming a new motor
      // acceleration of zero (simply integrate the last information).
      motor_vel = last_motor_vel_backwards_ + halfdt*(last_motor_acc_backwards_ + 0        );
      motor_pos = last_motor_pos_backwards_ + halfdt*(last_motor_vel_backwards_ + motor_vel);

      // Calculate the new joint position/velocity assuming a new joint
      // acceleration of zero, equivalent to an extremely large joint
      // mass (relatively to the motor).  This is also "fixed" in the
      // second half of the backward transmission after the simulator
      // has provided a new joint position/velocity.
      joint_vel = last_joint_vel_backwards_;
      joint_pos = last_joint_pos_backwards_ + halfdt*(last_joint_vel_backwards_ + joint_vel);

      // Calculate the spring force between the two masses.
      spring_force = (2.0*tau*(motor_vel - joint_vel) + (motor_pos - joint_pos)) / trans_compl_;

      // This gives us the new motor acceleration (still assuming no
      // joint acceleration).
      motor_acc = (motor_force - spring_force) * trans_compl_ / (tau*tau + 2.0*tau*halfdt + halfdt*halfdt);

      // Recalculate the motor position/velocity, using this new acceleration.
      motor_vel = last_motor_vel_backwards_ + halfdt*(last_motor_acc_backwards_ + motor_acc);
      motor_pos = last_motor_pos_backwards_ + halfdt*(last_motor_vel_backwards_ + motor_vel);

      // Recalculate the spring force.
      spring_force = (2.0*tau*(motor_vel - joint_vel) + (motor_pos - joint_pos)) / trans_compl_;
    }
  
  // The spring force becomes the force seen by the joint.
  js[0]->commanded_effort_ = spring_force;

  // Save the information that is to be used in the second half, i.e. in
  // propagatePositionBackwards().  This includes the motor force
  // (driving this cycle) and the time step (calculated here).
  halfdt_backwards_ = halfdt;
  motor_force_backwards_ = motor_force;
}


void PR2BeltCompensatorTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<pr2_hardware_interface::Actuator*>& as)
{
  // Check the arguments.  This acts only on a single actuator/joint.
  assert(as.size() == 1);
  assert(js.size() == 1);

  // Again (as in the first half of the backward transmission) simulate
  // the motor-mass / belt-spring-damper / joint-mass system.  And
  // again, all variables are in joint-space units.  Only this time use
  // the joint position/velocity provided by the simulator.
  double motor_pos, motor_vel, motor_acc;
  double joint_pos, joint_vel;
  double motor_force;
  double spring_force;
  double halfdt;

  // Get the time step and motor force from the first half of the
  // backward transmission.
  halfdt = halfdt_backwards_;
  motor_force = motor_force_backwards_;

  // Get the new joint position and velocity, as calculated by the
  // simulator.
  joint_pos = js[0]->position_ - js[0]->reference_position_;
  joint_vel = js[0]->velocity_;

  // As in the first half, if the transmission compliance or time
  // constant are zero, the transmission collapses to a regular rigid
  // transmission.
  if ((trans_compl_ == 0.0) || (trans_tau_ == 0.0))
    {
      // Immediately propagate the joint position/velocity to the motor,
      // ignoring the (infinitely fast) model dynamics.
      motor_acc = 0.0;
      motor_vel = joint_vel;
      motor_pos = joint_pos;
    }
  else
    {
      // Update the model.  Note for numerical stability, we again lower
      // bound the tranmission time constant by dt/2.
      double tau = (halfdt < trans_tau_ ? trans_tau_ : halfdt);

      // Calculate the new motor position/velocity assuming a new motor
      // acceleration of zero.
      motor_vel = last_motor_vel_backwards_ + halfdt*(last_motor_acc_backwards_ + 0        );
      motor_pos = last_motor_pos_backwards_ + halfdt*(last_motor_vel_backwards_ + motor_vel);

      // Calculate the spring force between the two masses.
      spring_force = (2.0*tau*(motor_vel - joint_vel) + (motor_pos - joint_pos)) / trans_compl_;

      // This gives us the new motor acceleration.
      motor_acc = (motor_force - spring_force) * trans_compl_ / (tau*tau + 2.0*tau*halfdt + halfdt*halfdt);

      // Recalculate the motor position/velocity, using this new acceleration.
      motor_vel = last_motor_vel_backwards_ + halfdt*(last_motor_acc_backwards_ + motor_acc);
      motor_pos = last_motor_pos_backwards_ + halfdt*(last_motor_vel_backwards_ + motor_vel);
    }

  // Save the current state for the next cycle.
  last_motor_pos_backwards_ = motor_pos;
  last_motor_vel_backwards_ = motor_vel;
  last_motor_acc_backwards_ = motor_acc;

  last_joint_pos_backwards_ = joint_pos;
  last_joint_vel_backwards_ = joint_vel;

  // Push the motor position/velocity to the actuator, accounting for
  // the mechanical reduction.
  as[0]->state_.position_ = motor_pos * mechanical_reduction_;
  as[0]->state_.velocity_ = motor_vel * mechanical_reduction_;

  // Also push the motor force to the actuator.  Note we already assumed
  // that the commanded actuator effort was accurately executed/applied,
  // so the measured actuator effort is just the motor force.
  as[0]->state_.last_measured_effort_ = motor_force / mechanical_reduction_;


  // By storing the new actuator data, we are advancing to the next
  // servo cycle.  Always make sure the timing has been initialized.
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


  // Simulate calibration sensors by filling out actuator states
  this->joint_calibration_simulator_.simulateJointCalibration(js[0],as[0]);
}

} // namespace
