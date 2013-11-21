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
 *   passive js->commanded_effort_ is 1/2?? of MT converted to joint torques

 * Example transmission block
 *
 * <transmission name="r_gripper_trans" type="pr2_mechanism_model/PR2GripperTransmission">
 *   <actuator name="r_gripper_motor"/>
 *   <gap_joint L0="0.0375528" a="0.0683698" b="0.0433849" gear_ratio="40.095" h="0.0"
 *              mechanical_reduction="1.0" name="r_gripper_joint" phi0="0.518518122146"
 *              r="0.0915" screw_reduction="0.004" t0="-0.0001914" theta0="0.0628824676201"/>
 *   <!-- if a gazebo joint exists as [l|r]_gripper_joint, use this tag to have
 *        gripper transmission apply torque directly to prismatic joint
 *        this should be the default behavior in diamondback, deprecating this flag -->
 *
 *   <!-- set passive joint angles so things look nice in rviz -->
 *   <passive_joint name="r_gripper_l_finger_joint"/>
 *   <passive_joint name="r_gripper_r_finger_joint"/>
 *   <passive_joint name="r_gripper_r_finger_tip_joint"/>
 *   <passive_joint name="r_gripper_l_finger_tip_joint"/>
 *
 *   <!-- screw joint used to actuate gripper
 *     TODO: rename "name" to something like actuated_screw_joint
 *   -->
 *   <simulated_actuated_joint name="r_gripper_motor_screw_joint"
 *                             passive_actuated_joint="r_gripper_motor_slider_joint"
 *                             simulated_reduction="3141.6"/>
 * </transmission>
 *
 */
#include "pr2_mechanism_model/pr2_gripper_transmission.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <numeric>
#include <angles/angles.h>
#include <boost/lexical_cast.hpp>

using namespace pr2_hardware_interface;
using namespace pr2_mechanism_model;

PLUGINLIB_EXPORT_CLASS(pr2_mechanism_model::PR2GripperTransmission,
                         pr2_mechanism_model::Transmission)

bool PR2GripperTransmission::initXml(TiXmlElement *config, Robot *robot)
{
  if (this->initXml(config))
  {
    // set robot actuator enabled
    for (std::vector<std::string>::iterator actuator_name = actuator_names_.begin(); actuator_name != actuator_names_.end(); ++actuator_name)
    {
      if (robot->getActuator(*actuator_name))
      {
        robot->getActuator(*actuator_name)->command_.enable_ = true;
      }
      else
      {
        ROS_ERROR("PR2GripperTransmission actuator named \"%s\" not loaded in Robot", actuator_name->c_str());
        return false;
      }
    }

    // look for joint_names_ in robot
    for (std::vector<std::string>::iterator joint_name = joint_names_.begin(); joint_name != joint_names_.end(); ++joint_name)
    {
      if (!robot->robot_model_.getJoint(*joint_name))
      {
        ROS_ERROR("PR2GripperTransmission joint named \"%s\" not loaded in Robot", joint_name->c_str());
        return false;
      }
    }

    // init successful
    return true;
  }
  else
    return false;
}

bool PR2GripperTransmission::initXml(TiXmlElement *config)
{
  const char *name = config->Attribute("name");
  name_ = name ? name : "";
  //myfile.open("transmission_data.txt");
  TiXmlElement *ael = config->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name)
  {
    ROS_ERROR("PR2GripperTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  actuator_names_.push_back(actuator_name);

  for (TiXmlElement *j = config->FirstChildElement("gap_joint"); j; j = j->NextSiblingElement("gap_joint"))
  {
    const char *gap_joint_name = j->Attribute("name");
    if (!gap_joint_name)
    {
      ROS_ERROR("PR2GripperTransmission did not specify joint name");
      return false;
    }
    gap_joint_ = std::string(gap_joint_name);
    joint_names_.push_back(gap_joint_name);

    // get the mechanical reduction
    const char *joint_reduction = j->Attribute("mechanical_reduction");
    if (!joint_reduction)
    {
      ROS_ERROR("PR2GripperTransmission's joint \"%s\" has no coefficient: mechanical reduction.", gap_joint_name);
      return false;
    }
    try
    {
      gap_mechanical_reduction_ = boost::lexical_cast<double>(joint_reduction);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("joint_reduction (%s) is not a float",joint_reduction);
      return false;
    }

    // get the screw drive reduction
    const char *screw_reduction_str = j->Attribute("screw_reduction");
    if (screw_reduction_str == NULL)
    {
      screw_reduction_ = 2.0/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: screw drive reduction, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        screw_reduction_ = boost::lexical_cast<double>(screw_reduction_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("screw_reduction (%s) is not a float",screw_reduction_str);
        return false;
      }

    // get the gear_ratio
    const char *gear_ratio_str = j->Attribute("gear_ratio");
    if (gear_ratio_str == NULL)
    {
      gear_ratio_ = 29.16;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: gear_ratio, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        gear_ratio_ = boost::lexical_cast<double>(gear_ratio_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("gear_ratio (%s) is not a float",gear_ratio_str);
        return false;
      }

    // get the theta0 coefficient
    const char *theta0_str = j->Attribute("theta0");
    if (theta0_str == NULL)
    {
      theta0_ = 2.97571*M_PI/180.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: theta0, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        theta0_ = boost::lexical_cast<double>(theta0_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("theta0 (%s) is not a float",theta0_str);
        return false;
      }
    // get the phi0 coefficient
    const char *phi0_str = j->Attribute("phi0");
    if (phi0_str == NULL)
    {
      phi0_ = 29.98717*M_PI/180.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: phi0, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        phi0_ = boost::lexical_cast<double>(phi0_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("phi0 (%s) is not a float",phi0_str);
        return false;
      }
    // get the t0 coefficient
    const char *t0_str = j->Attribute("t0");
    if (t0_str == NULL)
    {
      t0_ = -0.19543/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: t0, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        t0_ = boost::lexical_cast<double>(t0_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("t0 (%s) is not a float",t0_str);
        return false;
      }
    // get the L0 coefficient
    const char *L0_str = j->Attribute("L0");
    if (L0_str == NULL)
    {
      L0_ = 34.70821/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: L0, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        L0_ = boost::lexical_cast<double>(L0_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("L0 (%s) is not a float",L0_str);
        return false;
      }
    // get the h coefficient
    const char *h_str = j->Attribute("h");
    if (h_str == NULL)
    {
      h_ = 5.200/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: h, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        h_ = boost::lexical_cast<double>(h_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("h (%s) is not a float",h_str);
        return false;
      }
    // get the a coefficient
    const char *a_str = j->Attribute("a");
    if (a_str == NULL)
    {
      a_ = 67.56801/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: a, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        a_ = boost::lexical_cast<double>(a_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("a (%s) is not a float",a_str);
        return false;
      }
    // get the b coefficient
    const char *b_str = j->Attribute("b");
    if (b_str == NULL)
    {
      b_ = 48.97193/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: b, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        b_ = boost::lexical_cast<double>(b_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("b (%s) is not a float",b_str);
        return false;
      }
    // get the r coefficient
    const char *r_str = j->Attribute("r");
    if (r_str == NULL)
    {
      r_ = 91.50000/1000.0;
      ROS_WARN("PR2GripperTransmission's joint \"%s\" has no coefficient: r, using default for PR2 alpha2.", gap_joint_name);
    }
    else
      try
      {
        r_ = boost::lexical_cast<double>(r_str);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("r (%s) is not a float",r_str);
        return false;
      }
  }

  // Print all coefficients
  ROS_DEBUG("Gripper transmission parameters for %s: a=%f, b=%f, r=%f, h=%f, L0=%f, t0=%f, theta0=%f, phi0=%f, gear_ratio=%f, screw_red=%f",
            name_.c_str(), a_, b_, r_, h_, L0_, t0_, theta0_, phi0_, gear_ratio_, screw_reduction_);

  // Get passive joint informations
  for (TiXmlElement *j = config->FirstChildElement("passive_joint"); j; j = j->NextSiblingElement("passive_joint"))
  {
    const char *passive_joint_name = j->Attribute("name");
    if (!passive_joint_name)
    {
      ROS_ERROR("PR2GripperTransmission did not specify joint name");
      return false;
    }

    // add joint name to list
    // joint_names_.push_back(passive_joint_name);
    passive_joints_.push_back(passive_joint_name);
  }

  // Get screw joint informations
  for (TiXmlElement *j = config->FirstChildElement("simulated_actuated_joint"); j; j = j->NextSiblingElement("simulated_actuated_joint"))
  {
    const char *simulated_actuated_joint_name = j->Attribute("name");
    if (simulated_actuated_joint_name)
    {
      // joint_names_.push_back(simulated_actuated_joint_name);
    }
    else
    {
      ROS_ERROR("PR2GripperTransmission simulated_actuated_joint did snot specify joint name");
      return false;
    }

    // get the thread pitch
    const char *simulated_reduction = j->Attribute("simulated_reduction");
    if (!simulated_reduction)
    {
      ROS_ERROR("PR2GripperTransmission's simulated_actuated_joint \"%s\" has no coefficient: simulated_reduction.", simulated_actuated_joint_name);
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

    // get any additional joint introduced from this screw joint implementation
    // for the gripper, this is due to the limitation that screw constraint
    // requires axis of rotation to be aligned with line between CG's of the two
    // connected bodies.  For this reason, an additional slider joint was introduced
    // thus, requiring joint state to be published for motion planning packages
    // and that's why we're here.
    const char *passive_actuated_joint_name = j->Attribute("passive_actuated_joint");
    if (passive_actuated_joint_name)
    {
      has_simulated_passive_actuated_joint_ = true;
      // joint_names_.push_back(passive_actuated_joint_name);
    }
  }

  // assuming simulated gripper prismatic joint exists, use it

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
  double gap_size,double &MR,double &dMR_dtheta,double &dtheta_dt,double &dMR_dt)
{
    // get theta for jacobian calculation
    double sin_theta        = (gap_size - t0_)/r_ + sin(theta0_);
    sin_theta = sin_theta > 1.0 ? 1.0 : sin_theta < -1.0 ? -1.0 : sin_theta;
    double theta            = asin(sin_theta);

    // compute inverse transform for the gap joint, returns MR and dMR_dtheta
    inverseGapStates1(theta,MR,dMR_dtheta,dtheta_dt,dMR_dt);
}

///////////////////////////////////////////////////////////
/// inverse of computeGapStates()
/// need theta as input
/// computes MR, dMR_dtheta, dtheta_dt, dMR_dt
void PR2GripperTransmission::inverseGapStates1(
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


///////////////////////////////////////////////////////////
/// assign joint position, velocity, effort from actuator state
/// all passive joints are assigned by single actuator state through mimic?
void PR2GripperTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{

  ROS_ASSERT(as.size() == 1);
  // js has passive joints and 1 gap joint and 1 screw joint
#if 0
  if (has_simulated_passive_actuated_joint_) {ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 2);}
  ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 1);
#endif

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
  js[0]->position_        = gap_size*2.0; // function engineering's transmission give half the total gripper size
  js[0]->velocity_        = gap_velocity*2.0;
  js[0]->measured_effort_ = gap_effort/2.0;
  //ROS_ERROR("prop pos eff=%f",js[0]->measured_effort_);

#if 0
  // Determines the states of the passive joints.
  // we need to do this for each finger, in simulation, each finger has it's state filled out
  for (size_t i = 1; i < passive_joints_.size()+1; ++i)
  {
    js[i]->position_           = theta - theta0_;
    js[i]->velocity_           = dtheta_dMR * MR_dot;
    js[i]->measured_effort_    = MT / dtheta_dMR / RAD2MR;
    js[i]->reference_position_ = MT / dtheta_dMR / RAD2MR;
  }

  // screw joint state is not important to us, fill with zeros
  js[passive_joints_.size()+1]->position_           = 0.0;
  js[passive_joints_.size()+1]->velocity_           = 0.0;
  js[passive_joints_.size()+1]->measured_effort_    = 0.0;
  js[passive_joints_.size()+1]->reference_position_ = 0.0;
  js[passive_joints_.size()+1]->calibrated_         = true; // treat passive simulation joints as "calibrated"

  if (has_simulated_passive_actuated_joint_)
  {
    // screw joint state is not important to us, fill with zeros
    js[passive_joints_.size()+2]->position_           = 0.0;
    js[passive_joints_.size()+2]->velocity_           = 0.0;
    js[passive_joints_.size()+2]->measured_effort_    = 0.0;
    js[passive_joints_.size()+2]->reference_position_ = 0.0;
    js[passive_joints_.size()+2]->calibrated_         = true; // treat passive simulation joints as "calibrated"
  }
#endif
}

// this is needed for simulation, so we can recover encoder value given joint angles
void PR2GripperTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == 1);
  ROS_DEBUG("js [%d], pjs [%d]", js.size(), passive_joints_.size());

  // keep the simulation stable by using the minimum rate joint to compute gripper gap rate
  double MR,dMR_dtheta,dtheta_dt,dMR_dt;
  double joint_rate;
  {
    // new gripper model has an actual physical slider joint in simulation
    // use the new slider joint for determining gipper position, so forward/backward are consistent
    double gap_size         = js[0]->position_/2.0; // js position should be normalized
    // compute inverse transform for the gap joint, returns MR and dMR_dtheta
    inverseGapStates(gap_size,MR,dMR_dtheta,dtheta_dt,dMR_dt);
    double gap_rate         = js[0]->velocity_/2.0;
    joint_rate              = gap_rate * dtheta_dt;
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
  as[0]->state_.last_measured_effort_ = 2.0*gap_effort / dMR_dt * RAD2MR * gap_mechanical_reduction_;

  // Update the timing (making sure it's initialized).
  if (!simulated_actuator_timestamp_initialized_)
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

void PR2GripperTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
#if 0
  if (has_simulated_passive_actuated_joint_) {ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 2);}
  else {ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 1);}
#endif

  //
  // in hardware, the position of passive joints are set by propagatePosition, so they should be identical and
  // the inverse transform should be consistent.
  // note for simulation:
  //   new gripper model has an actual physical slider joint in simulation
  //   use the new slider joint for determining gipper position, so forward/backward are consistent
  double gap_size         = js[0]->position_/2.0; // js position should be normalized

  // now do the reverse transform
  double MR,dMR_dtheta,dtheta_dt,dMR_dt;
  // compute inverse transform for the gap joint, returns MR and dMR_dtheta
  inverseGapStates(gap_size,MR,dMR_dtheta,dtheta_dt,dMR_dt);

  double gap_effort       = js[0]->commanded_effort_; /// Newtons

  //ROS_ERROR("prop eff eff=%f",gap_effort);

  /// actuator commanded effort = gap_dffort / dMR_dt / (2*pi)  * gap_mechanical_reduction_
  as[0]->command_.enable_ = true;
  as[0]->command_.effort_ = 2.0*gap_effort / dMR_dt * RAD2MR * gap_mechanical_reduction_;
}

void PR2GripperTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  ROS_ASSERT(as.size() == 1);
#if 0
  ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 1);
#endif
  ROS_ASSERT(simulated_reduction_>0.0);

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

#if 0
  {
    // propagate fictitious joint effort backwards
    // ROS_ERROR("prop eff back eff=%f",js[0]->commanded_effort_);

    // set screw joint effort if simulated
    js[passive_joints_.size()+1]->commanded_effort_  = gap_effort/simulated_reduction_;
  }
#endif
}

