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
 * Author: Stuart Glaser
 */

/*
 * propagatePosition (from as to js)
 *   as position and velocity are doubled to get js, since gripper is two sided
 *   as torque is directly converted to gap_effort
 *   as torque to passive joint is /4 since there are 4 links
 * propagateEffort (from js to as)
 *   popluate only as->commanded_.effort_
 *     this is directly transferred as torque and gap effort is 1to1
 *
 * below only for simulation
 *
 * propagatePositionBackwards (from js to as)
 *   as position and velocity transfers 1to1 to joint_angle (1-sided)
 *   as last_measured_effort_ should be 1to1 gap_effort of non-passive js->commanded_effort
 * propagateEffortBackwards
 *   non-passive js->commanded_effort_ is 1to1 with MT
 *   passive js->commanded_effort_ is 1/4?? of MT converted to joint torques
 */
#include "pr2_mechanism_model/pr2_gripper_transmission.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <numeric>
#include <angles/angles.h>

using namespace pr2_hardware_interface;
using namespace pr2_mechanism_model;

PLUGINLIB_REGISTER_CLASS(PR2GripperTransmission,
                         pr2_mechanism_model::PR2GripperTransmission,
                         pr2_mechanism_model::Transmission)

bool PR2GripperTransmission::initXml(TiXmlElement *config, Robot *robot)
{
  const char *name = config->Attribute("name");
  name_ = name ? name : "";
  //myfile.open("transmission_data.txt");
  TiXmlElement *ael = config->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name || !robot->getActuator(actuator_name))
  {
    ROS_ERROR("PR2GripperTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  robot->getActuator(actuator_name)->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);

  for (TiXmlElement *j = config->FirstChildElement("gap_joint"); j; j = j->NextSiblingElement("gap_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name)
    {
      ROS_ERROR("PR2GripperTransmission did not specify joint name");
      return false;
    }

    const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
    if (!joint)
    {
      ROS_ERROR("PR2GripperTransmission could not find joint named \"%s\"", joint_name);
      return false;
    }
    gap_joint_ = std::string(joint_name);
    joint_names_.push_back(joint_name);  // The first joint is the gap joint

    // get the mechanical reduction
    const char *joint_reduction = j->Attribute("mechanical_reduction");
    if (!joint_reduction)
    {
      ROS_ERROR("PR2GripperTransmission's joint \"%s\" has no coefficient: mechanical reduction.", joint_name);
      return false;
    }
    gap_mechanical_reduction_ = atof(joint_reduction);

    // get the screw drive reduction
    const char *screw_reduction_str = j->Attribute("screw_reduction");
    if (screw_reduction_str == NULL)
    {
      screw_reduction_ = 2.0/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: screw drive reduction, using default for PR2 alpha2.", joint_name);
    }
    else
      screw_reduction_ = atof(screw_reduction_str);
    //ROS_INFO("screw drive reduction. %f", screw_reduction_);

    // get the gear_ratio
    const char *gear_ratio_str = j->Attribute("gear_ratio");
    if (gear_ratio_str == NULL)
    {
      gear_ratio_ = 29.16;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: gear_ratio, using default for PR2 alpha2.", joint_name);
    }
    else
      gear_ratio_ = atof(gear_ratio_str);
    //ROS_INFO("gear_ratio. %f", gear_ratio_);

    // get the theta0 coefficient
    const char *theta0_str = j->Attribute("theta0");
    if (theta0_str == NULL)
    {
      theta0_ = 2.97571*M_PI/180.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: theta0, using default for PR2 alpha2.", joint_name);
    }
    else
      theta0_ = atof(theta0_str);
    // get the phi0 coefficient
    const char *phi0_str = j->Attribute("phi0");
    if (phi0_str == NULL)
    {
      phi0_ = 29.98717*M_PI/180.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: phi0, using default for PR2 alpha2.", joint_name);
    }
    else
      phi0_ = atof(phi0_str);
    // get the t0 coefficient
    const char *t0_str = j->Attribute("t0");
    if (t0_str == NULL)
    {
      t0_ = -0.19543/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: t0, using default for PR2 alpha2.", joint_name);
    }
    else
      t0_ = atof(t0_str);
    // get the L0 coefficient
    const char *L0_str = j->Attribute("L0");
    if (L0_str == NULL)
    {
      L0_ = 34.70821/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: L0, using default for PR2 alpha2.", joint_name);
    }
    else
      L0_ = atof(L0_str);
    // get the h coefficient
    const char *h_str = j->Attribute("h");
    if (h_str == NULL)
    {
      h_ = 5.200/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: h, using default for PR2 alpha2.", joint_name);
    }
    else
      h_ = atof(h_str);
    // get the a coefficient
    const char *a_str = j->Attribute("a");
    if (a_str == NULL)
    {
      a_ = 67.56801/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: a, using default for PR2 alpha2.", joint_name);
    }
    else
      a_ = atof(a_str);
    // get the b coefficient
    const char *b_str = j->Attribute("b");
    if (b_str == NULL)
    {
      b_ = 48.97193/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: b, using default for PR2 alpha2.", joint_name);
    }
    else
      b_ = atof(b_str);
    // get the r coefficient
    const char *r_str = j->Attribute("r");
    if (r_str == NULL)
    {
      r_ = 91.50000/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: r, using default for PR2 alpha2.", joint_name);
    }
    else
      r_ = atof(r_str);
  }

  // Print all coefficients
  ROS_DEBUG("Gripper transmission parameters for %s: a=%f, b=%f, r=%f, h=%f, L0=%f, t0=%f, theta0=%f, phi0=%f, gear_ratio=%f, screw_red=%f",
            name_.c_str(), a_, b_, r_, h_, L0_, t0_, theta0_, phi0_, gear_ratio_, screw_reduction_);

  // Check if flag is set to skip application of torque to passive joints in simulation
  if (config->FirstChildElement("use_simulated_slider_joint"))
    use_simulated_slider_joint_ = true;

  // Get passive joint informations
  for (TiXmlElement *j = config->FirstChildElement("passive_joint"); j; j = j->NextSiblingElement("passive_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name)
    {
      ROS_ERROR("PR2GripperTransmission did not specify joint name");
      return false;
    }
    const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);

    if (!joint)
    {
      ROS_ERROR("PR2GripperTransmission could not find joint named \"%s\"", joint_name);
      return false;
    }

    // add joint name to list
    joint_names_.push_back(joint_name);  // Adds the passive joints after the gap joint
    passive_joints_.push_back(joint_name);
  }
  return true;
}

///////////////////////////////////////////////////////////
/// given actuator states (motor revolustion, joint torques), compute gap properties.
void PR2GripperTransmission::computeGapStates(
  double MR,double MR_dot,double MT,
  double &theta,double &dtheta_dMR,double &dt_dtheta,double &dt_dMR,double &gap_size,double &gap_velocity,double &gap_effort)
{
  //
  // below transforms from encoder value to gap size, based on 090224_link_data.xls provided by Functions Engineering
  //
  double u            = (a_*a_+b_*b_-h_*h_
                         -pow(L0_+MR*screw_reduction_/gear_ratio_,2))/(2.0*a_*b_);
  u                   = u < -1.0 ? -1.0 : u > 1.0 ? 1.0 : u;
  theta               = theta0_ - phi0_ + acos(u);
  // limit theta
  //theta = theta > 0 ? theta : 0;
  // force theta to be greater than theta0_
  //theta = angles::normalize_angle_positive(angles::shortest_angular_distance(theta0_,theta))+theta0_;

  gap_size            = t0_ + r_ * ( sin(theta) - sin(theta0_) );

  //
  // compute jacobians based on transforms, get the velocity of the gripper gap size based on encoder velocity
  //
  // for jacobian, we want to limit MR >= 0
  MR = MR >= 0.0 ? MR : 0.0;
  // then recompute u and theta based on restricted MR
  u                   = (a_*a_+b_*b_-h_*h_
                         -pow(L0_+MR*screw_reduction_/gear_ratio_,2))/(2.0*a_*b_);
  u                   = u < -1.0 ? -1.0 : u > 1.0 ? 1.0 : u;
  double tmp_theta    = theta0_ - phi0_ + acos(u);

  //
  double arg          = 1.0 - pow(u,2);
  arg                 = arg > TOL ? arg : TOL; //LIMIT: CAP u at TOL artificially

  double du_dMR       = -(L0_ * screw_reduction_)/(gear_ratio_*a_*b_) // d(arg)/d(MR)
                        -MR/(a_*b_)*pow(screw_reduction_/gear_ratio_,2);

  dtheta_dMR          = -1.0/sqrt(arg) * du_dMR; // derivative of acos

  dt_dtheta           = r_ * cos( tmp_theta );
  dt_dMR              = dt_dtheta * dtheta_dMR;
  gap_velocity        = MR_dot * dt_dMR;

  //
  // get the effort at the gripper gap based on torque at the motor
  // gap effort = motor torque         * dmotor_theta/dt
  //            = MT                   * dmotor_theta_dt
  //            = MT                   * dMR_dt          / (2*pi)
  //            = MT                   / dt_dMR          * 2*pi
  //
  gap_effort          = MT      / dt_dMR / RAD2MR ;
  //ROS_WARN("debug: %f %f %f",gap_effort,MT,dt_dMR,RAD2MR);
}

///////////////////////////////////////////////////////////
/// inverse of computeGapStates()
/// need theta as input
/// computes MR, dMR_dtheta, dtheta_dt, dMR_dt
void PR2GripperTransmission::inverseGapStates(
  double theta,double &MR,double &dMR_dtheta,double &dtheta_dt,double &dMR_dt)
{
  // limit theta
  //theta = theta > 0 ? theta : 0;
  // force theta to be greater than theta0_
  //theta = angles::normalize_angle_positive(angles::shortest_angular_distance(theta0_,theta))+theta0_;

  // now do the reverse transform
  double arg         = -2.0*a_*b_*cos(theta-theta0_+phi0_)
                                   -h_*h_+a_*a_+b_*b_;
  if (arg > 0.0)
  {
    MR               = gear_ratio_/screw_reduction_ * ( sqrt(arg) - L0_ );
    dMR_dtheta       = gear_ratio_/(2.0 * screw_reduction_) / sqrt(arg)
                       * 2.0 * a_ * b_ * sin(theta + phi0_ - theta0_);
  }
  else
  {
    MR               = gear_ratio_/screw_reduction_ * ( 0.0       - L0_ );
    dMR_dtheta       = 0.0;
  }

  // compute gap_size from theta
  double gap_size = t0_ + r_ * ( sin(theta) - sin(theta0_) ); // in mm

  // compute inverse jacobians for the transform
  // for this, enforce dMR_dtheta >= 0
  // since there are two roots, take the positive root
  // @todo: this affects sim only, need to check this for sim.
  double tmp_dMR_dtheta = fabs(dMR_dtheta);

  double u           = (gap_size - t0_)/r_ + sin(theta0_);
  double arg2        = 1.0 - pow(u,2);
  arg2               = arg2 > TOL ? arg2 : TOL; //LIMIT: CAP arg2 at TOL artificially
  dtheta_dt          = 1.0 / sqrt( arg2 ) / r_;  // derivative of asin
  dMR_dt             = tmp_dMR_dtheta * dtheta_dt;  // remember, here, t is gap_size
}

void PR2GripperTransmission::getRateFromMaxRateJoint(
  std::vector<JointState*>& js, std::vector<Actuator*>& as,
  int &max_rate_joint_index,double &rate)
{

  // obtain the physical location of passive joints in sim, and average them
  double max_rate   = -1.0; // a small initial value
  max_rate_joint_index = js.size();



  // Loops through the passive joints.
  for (size_t i = 1; i < js.size(); ++i)
  {
    if (fabs(js[i]->velocity_) > max_rate)
    {
      max_rate = fabs(js[i]->velocity_);
      max_rate_joint_index = i;

    }
  }
  if (max_rate_joint_index >= (int)js.size())
    ROS_ERROR("PR2 Gripper Transmission could not find a passive joint with minimum rate, mostly likely finger joints have exploded.  js[0:4]->velocity_=(%f,%f,%f,%f,%f)",js[0]->velocity_,js[1]->velocity_,js[2]->velocity_,js[3]->velocity_,js[4]->velocity_);

  rate = js[max_rate_joint_index]->velocity_;
}


///////////////////////////////////////////////////////////
/// assign joint position, velocity, effort from actuator state
/// all passive joints are assigned by single actuator state through mimic?
void PR2GripperTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{

  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1); // passive joints and 1 gap joint

  /// \brief motor revolutions = encoder value * gap_mechanical_reduction_ * RAD2MR
  ///        motor revolutions =      motor angle(rad)                     / (2*pi)
  ///                          =      theta                                / (2*pi)
  double MR        = as[0]->state_.position_ / gap_mechanical_reduction_ * RAD2MR ;

  /// \brief motor revolustions per second = motor angle rate (rad per second) / (2*pi)
  double MR_dot    = as[0]->state_.velocity_ / gap_mechanical_reduction_ * RAD2MR ;

  ///
  ///  old MT definition - obsolete
  ///
  ///        but we convert it to Nm*(MR/rad)
  ///        motor torque = actuator_state->last_meausured_effort
  ///        motor torque = I * theta_ddot
  ///        MT           = I * MR_ddot  (my definition)
  ///                     = I * theta_ddot / (2*pi)
  ///                     = motot torque   / (2*pi)
  //double MT        = as[0]->state_.last_measured_effort_ / gap_mechanical_reduction_ * RAD2MR ;


  /// \brief gripper motor torque: received from hardware side in newton-meters
  double MT        = as[0]->state_.last_measured_effort_ / gap_mechanical_reduction_;

  /// internal theta state, gripper closed it is theta0_.  same as finger joint angles + theta0_.
  double theta, dtheta_dMR, dt_dtheta, dt_dMR;
  /// information on the fictitious joint: gap_joint
  double gap_size,gap_velocity,gap_effort;

  // compute gap position, velocity, measured_effort from actuator states
  computeGapStates(MR,MR_dot,MT,theta,dtheta_dMR, dt_dtheta, dt_dMR,gap_size,gap_velocity,gap_effort);

  // Determines the state of the gap joint.
  js[0]->position_       = gap_size*2.0; // function engineering's transmission give half the total gripper size
  js[0]->velocity_       = gap_velocity*2.0;
  js[0]->measured_effort_ = gap_effort;
  //ROS_ERROR("prop pos eff=%f",js[0]->measured_effort_);

  // Determines the states of the passive joints.
  // we need to do this for each finger, in simulation, each finger has it's state filled out
  for (size_t i = 1; i < js.size(); ++i)
  {
    js[i]->position_       = theta - theta0_;
    js[i]->velocity_       = dtheta_dMR * MR_dot;
    js[i]->measured_effort_ = MT / dtheta_dMR / RAD2MR;
  }
}

// this is needed for simulation, so we can recover encoder value given joint angles
void PR2GripperTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  // keep the simulation stable by using the minimum rate joint to compute gripper gap rate
  double MR,dMR_dtheta,dtheta_dt,dMR_dt;
  double joint_angle, joint_rate;
  if (use_simulated_slider_joint_)
  {
    // new gripper model has an actual physical slider joint in simulation
    // use the new slider joint for determining gipper position, so forward/backward are consistent
    double gap_size         = js[0]->position_/2.0; // js position should be normalized
    // get theta for jacobian calculation
    double sin_theta        = (gap_size - t0_)/r_ + sin(theta0_);
    sin_theta = sin_theta > 1.0 ? 1.0 : sin_theta < -1.0 ? -1.0 : sin_theta;
    double theta            = asin(sin_theta);

    // compute inverse transform for the gap joint, returns MR and dMR_dtheta
    inverseGapStates(theta,MR,dMR_dtheta,dtheta_dt,dMR_dt);
    double gap_rate         = js[0]->velocity_/2.0;
    joint_rate              = gap_rate * dtheta_dt;
  }
  else
  {
    // use the same passive joint for determining gipper position, so forward/backward are consistent
    joint_angle             = js[default_passive_joint_index_from_sim]->position_; // js position should be normalized
    joint_rate              = js[default_passive_joint_index_from_sim]->velocity_;
    // recover gripper intermediate variable theta from joint angle
    double theta            = angles::shortest_angular_distance(theta0_,joint_angle)+2.0*theta0_;
    // compute inverse transform for the gap joint, returns MR and dMR_dtheta
    inverseGapStates(theta,MR,dMR_dtheta,dtheta_dt,dMR_dt);
  }
  double gap_effort         = js[0]->commanded_effort_;

  //ROS_ERROR("prop pos back eff=%f",gap_effort);

  /// should be exact inverse of propagatePosition() call
  as[0]->state_.position_             = MR                        * gap_mechanical_reduction_ / RAD2MR ;

  /// state velocity                  = MR_dot                    * gap_mechanical_reduction_ / rad2mr
  ///                                 = theta_dot    * dMR_dtheta * gap_mechanical_reduction_ / rad2mr
  as[0]->state_.velocity_             = joint_rate   * dMR_dtheta * gap_mechanical_reduction_ / RAD2MR ;

  /// motor torque                    = inverse of getting gap effort from motor torque
  ///                                 = gap_effort * dt_dMR / (2*pi)  * gap_mechanical_reduction_
  ///                                 = gap_effort / dMR_dt * RAD2MR * gap_mechanical_reduction_
  as[0]->state_.last_measured_effort_ = gap_effort / dMR_dt * RAD2MR * gap_mechanical_reduction_;

}

void PR2GripperTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  //
  // in hardware, the position of passive joints are set by propagatePosition, so they should be identical and
  // the inverse transform should be consistent.
  //
  // using the min rate joint for the sake of sim stability before true non-dynamics mimic is implemented
  //
  double joint_angle, joint_rate, joint_torque;
  // use the same passive joint for determining gipper position, so forward/backward are consistent
  joint_angle             = js[default_passive_joint_index_from_sim]->position_; // js position should be normalized
  joint_rate              = js[default_passive_joint_index_from_sim]->velocity_;
  joint_torque            = js[default_passive_joint_index_from_sim]->measured_effort_;

  // recover gripper intermediate variable theta from joint angle
  double theta            = angles::shortest_angular_distance(theta0_,joint_angle)+2.0*theta0_;

  // now do the reverse transform
  double MR,dMR_dtheta,dtheta_dt,dMR_dt;
  // compute inverse transform for the gap joint, returns MR and dMR_dtheta
  inverseGapStates(theta,MR,dMR_dtheta,dtheta_dt,dMR_dt);

  double gap_effort             = js[0]->commanded_effort_; /// Newtons

  //ROS_ERROR("prop eff eff=%f",gap_effort);

  /// actuator commanded effort = gap_dffort / dMR_dt / (2*pi)  * gap_mechanical_reduction_
  as[0]->command_.effort_       = gap_effort / dMR_dt * RAD2MR * gap_mechanical_reduction_;
}

void PR2GripperTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == passive_joints_.size() + 1);

  //
  // below transforms from encoder value to gap size, based on 090224_link_data.xls provided by Functions Engineering
  //
  /// \brief taken from propagatePosition()
  double MR        = as[0]->state_.position_ / gap_mechanical_reduction_ * RAD2MR ;
  double MR_dot    = as[0]->state_.velocity_ / gap_mechanical_reduction_ * RAD2MR ;
  double MT        = as[0]->command_.effort_ / gap_mechanical_reduction_;

  /// internal theta state, gripper closed it is theta0_.  same as finger joint angles + theta0_.
  double theta, dtheta_dMR, dt_dtheta, dt_dMR;
  /// information on the fictitious joint: gap_joint
  double gap_size,gap_velocity,gap_effort;

  // compute gap position, velocity, measured_effort from actuator states
  computeGapStates(MR,MR_dot,MT,theta,dtheta_dMR, dt_dtheta, dt_dMR,gap_size,gap_velocity,gap_effort);

  // propagate fictitious joint effort backwards
  double eps=0.01;
  js[0]->commanded_effort_  = (1.0-eps)*js[0]->commanded_effort_ + eps*gap_effort;
  //ROS_ERROR("prop eff back eff=%f",js[0]->commanded_effort_);

  if (!use_simulated_slider_joint_)
  for (size_t i = 1; i < js.size(); ++i)
  {

    // enforce all gripper positions based on gap position
    // check to see how off each finger link is

    // now do the reverse transform
    // get individual passive joint error
    //double joint_theta              = theta0_ + angles::normalize_angle_positive(js[i]->position_);
    double joint_theta              = angles::shortest_angular_distance(theta0_,js[i]->position_) + 2.0*theta0_;
    // now do the reverse transform
    double joint_MR,joint_dMR_dtheta,joint_dtheta_dt,joint_dMR_dt;
    // compute inverse transform for the gap joint, returns MR and dMR_dtheta
    inverseGapStates(joint_theta,joint_MR,joint_dMR_dtheta,joint_dtheta_dt,joint_dMR_dt);


    // @todo: due to high transmission ratio, MT is too large for the damping available
    // with the given time step size in sim, so until implicit damping is implemented,
    // we'll scale MT with inverse of velocity to some power
    int max_joint_rate_index;
    double scale=1.0,rate_threshold=0.5;
    double max_joint_rate;
    getRateFromMaxRateJoint(js, as, max_joint_rate_index, max_joint_rate);
    if (fabs(max_joint_rate)>rate_threshold) scale /= pow(fabs(max_joint_rate/rate_threshold),2.0);
    //std::cout << "rate " << max_joint_rate << " absrate " << fabs(max_joint_rate) << " scale " << scale << std::endl;
    // sum joint torques from actuator motor and mimic constraint and convert to joint torques
    double eps2=0.01;
    js[i]->commanded_effort_ = (1.0-eps2)*js[i]->commanded_effort_ + eps2*scale*MT / dtheta_dMR;
    //ROS_ERROR("prop eff back eff[%d]=%f",i,js[i]->commanded_effort_);
  }
}
