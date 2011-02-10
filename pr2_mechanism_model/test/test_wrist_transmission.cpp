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

#include <gtest/gtest.h>
#include <vector>
#include "pr2_mechanism_model/wrist_transmission.h"
#include "pr2_hardware_interface/hardware_interface.h"

using namespace pr2_mechanism_model;
using namespace pr2_hardware_interface;
using namespace std;

static const double eps = 1e-6;

class BaseFixture : public testing::Test
{
public:
  WristTransmission wrist_;
  std::vector<Actuator*>   actuators_;
  std::vector<JointState*> joint_states_;

protected:
  //virtual void SetUp()
  BaseFixture()
  {
    actuators_.resize(2);
    actuators_[0] = new Actuator();
    actuators_[1] = new Actuator();

    joint_states_.resize(2);
    joint_states_[0] = new JointState();
    joint_states_[1] = new JointState();

    boost::shared_ptr<const urdf::Joint> joint(new urdf::Joint());
    joint_states_[0]->joint_ = joint;
    joint_states_[1]->joint_ = joint;
    
  }

  virtual void TearDown()
  {
    delete actuators_[0];
    delete actuators_[1];
    delete joint_states_[0];
    delete joint_states_[1];
  }
};

// ******* Sets all the gearings to 1. This is a very basic test *******
class EasyNoGearingTest : public BaseFixture
{
  virtual void SetUp()
  {
    vector<double> ar(2);
    ar[0] = 1.0;
    ar[1] = 1.0;

    vector<double> jr(2);
    jr[0] = 1.0;
    jr[1] = 1.0;

    wrist_.setReductions(ar, jr);
  }
};

TEST_F(EasyNoGearingTest, ForwardPositionTest1)
{
  actuators_[0]->state_.position_ = 1.0;
  actuators_[1]->state_.position_ = 1.0;
  wrist_.propagatePosition(actuators_, joint_states_);
  EXPECT_NEAR(joint_states_[0]->position_,  0.0, eps);
  EXPECT_NEAR(joint_states_[1]->position_, -1.0, eps);
}

TEST_F(EasyNoGearingTest, ForwardVelocityTest1)
{
  actuators_[0]->state_.velocity_ = 1.0;
  actuators_[1]->state_.velocity_ =-1.0;
  wrist_.propagatePosition(actuators_, joint_states_);
  EXPECT_NEAR(joint_states_[0]->velocity_,  1.0, eps);
  EXPECT_NEAR(joint_states_[1]->velocity_,  0.0, eps);
}

TEST_F(EasyNoGearingTest, ForwardMeasuredEffortTest1)
{
  actuators_[0]->state_.last_measured_effort_ =  1.0;
  actuators_[1]->state_.last_measured_effort_ = -1.0;
  wrist_.propagatePosition(actuators_, joint_states_);
  EXPECT_NEAR(joint_states_[0]->measured_effort_, 2.0, eps);
  EXPECT_NEAR(joint_states_[1]->measured_effort_, 0.0, eps);
}

// ******* Check joint gearing *******
class EasyJointGearingTest : public BaseFixture
{
  virtual void SetUp()
  {
    vector<double> ar(2);
    ar[0] = 1.0;
    ar[1] = 1.0;

    vector<double> jr(2);
    jr[0] = 10.0;
    jr[1] = 100.0;

    wrist_.setReductions(ar, jr);
  }
};

TEST_F(EasyJointGearingTest, ForwardPositionTest1)
{
  actuators_[0]->state_.position_ = 3.0;
  actuators_[1]->state_.position_ = 1.0;
  wrist_.propagatePosition(actuators_, joint_states_);
  EXPECT_NEAR(joint_states_[0]->position_,  0.10, eps);
  EXPECT_NEAR(joint_states_[1]->position_, -0.02, eps);
}

// ******* Check motor gearing *******
class EasyMotorGearingTest : public BaseFixture
{
  virtual void SetUp()
  {
    vector<double> ar(2);
    ar[0] = 10.0;
    ar[1] = 100.0;

    vector<double> jr(2);
    jr[0] = 1.0;
    jr[1] = 1.0;

    wrist_.setReductions(ar, jr);
  }
};

TEST_F(EasyMotorGearingTest, ForwardPositionTest1)
{
  actuators_[0]->state_.position_ = 100.0;
  actuators_[1]->state_.position_ = 0.0;
  wrist_.propagatePosition(actuators_, joint_states_);
  EXPECT_NEAR(joint_states_[0]->position_,  5, eps);
  EXPECT_NEAR(joint_states_[1]->position_, -5, eps);
}

TEST_F(EasyMotorGearingTest, ForwardPositionTest2)
{
  actuators_[0]->state_.position_ = 0.0;
  actuators_[1]->state_.position_ = 100.0;
  wrist_.propagatePosition(actuators_, joint_states_);
  EXPECT_NEAR(joint_states_[0]->position_, -0.5, eps);
  EXPECT_NEAR(joint_states_[1]->position_, -0.5, eps);
}

// ******* Check both directions *******
class PropagateSanityCheck : public BaseFixture
{
public:
  std::vector<Actuator*> actuators2_;

  PropagateSanityCheck()
  {
    actuators2_.resize(2);
    actuators2_[0] = new Actuator();
    actuators2_[1] = new Actuator();
  }

  ~PropagateSanityCheck()
  {
    delete actuators2_[0];
    delete actuators2_[1];
  }

  virtual void SetUp()
  {
    vector<double> ar(2);
    ar[0] = 10.0;
    ar[1] = 100.0;

    vector<double> jr(2);
    jr[0] = 20.0;
    jr[1] = 40.0;

    wrist_.setReductions(ar, jr);

    actuators_[0]->state_.velocity_ = 3.0;
    actuators_[1]->state_.velocity_ = 4.0;
    actuators_[0]->state_.last_measured_effort_ = 5.0;
    actuators_[1]->state_.last_measured_effort_ = 6.0;
  }
};

TEST_F(PropagateSanityCheck, Position)
{
  actuators_[0]->state_.position_ = 1.0;
  actuators_[1]->state_.position_ = 2.0;
  wrist_.propagatePosition(actuators_, joint_states_);
  wrist_.propagatePositionBackwards(joint_states_, actuators2_);
  EXPECT_NEAR(actuators2_[0]->state_.position_, actuators_[0]->state_.position_, eps);
  EXPECT_NEAR(actuators2_[1]->state_.position_, actuators_[1]->state_.position_, eps);
}

TEST_F(PropagateSanityCheck, Velocity)
{
  actuators_[0]->state_.velocity_ = 1.0;
  actuators_[1]->state_.velocity_ = 2.0;
  wrist_.propagatePosition(actuators_, joint_states_);
  wrist_.propagatePositionBackwards(joint_states_, actuators2_);
  EXPECT_NEAR(actuators2_[0]->state_.velocity_, actuators_[0]->state_.velocity_, eps);
  EXPECT_NEAR(actuators2_[1]->state_.velocity_, actuators_[1]->state_.velocity_, eps);
}

TEST_F(PropagateSanityCheck, MeasuredEffort)
{
  actuators_[0]->state_.last_measured_effort_ = 1.0;
  actuators_[1]->state_.last_measured_effort_ = 2.0;
  wrist_.propagatePosition(actuators_, joint_states_);
  wrist_.propagatePositionBackwards(joint_states_, actuators2_);
  EXPECT_NEAR(actuators2_[0]->state_.last_measured_effort_, actuators_[0]->state_.last_measured_effort_, eps);
  EXPECT_NEAR(actuators2_[1]->state_.last_measured_effort_, actuators_[1]->state_.last_measured_effort_, eps);
}

TEST_F(PropagateSanityCheck, CommandEffort)
{
  actuators_[0]->command_.effort_ = 1.0;
  actuators_[1]->command_.effort_ = 2.0;
  wrist_.propagateEffortBackwards(actuators_, joint_states_);
  wrist_.propagateEffort(joint_states_, actuators2_);
  EXPECT_NEAR(actuators2_[0]->command_.effort_, actuators_[0]->command_.effort_, eps);
  EXPECT_NEAR(actuators2_[1]->command_.effort_, actuators_[1]->command_.effort_, eps);
}


int main(int argc, char **argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
