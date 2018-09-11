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

/* Author: Wim Meeussen */

#include <string>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include "pr2_mechanism_model/joint_calibration_simulator.h"

using namespace pr2_mechanism_model;

int g_argc;
char** g_argv;

class TestParser : public testing::Test
{
public:

  /// constructor
  TestParser() {}

  /// Destructor
  ~TestParser() {}
};




TEST_F(TestParser, test)
{
  ASSERT_TRUE(g_argc == 2);

  urdf::Model robot;
  ASSERT_TRUE(robot.initFile(g_argv[1]));

#if URDFDOM_1_0_0_API
  urdf::JointConstSharedPtr jnt_cont1 = robot.getJoint("continuous1");
  urdf::JointConstSharedPtr jnt_cont2 = robot.getJoint("continuous2");
  urdf::JointConstSharedPtr jnt_rev  = robot.getJoint("revolute");
#else
  boost::shared_ptr<const urdf::Joint> jnt_cont1 = robot.getJoint("continuous1");
  boost::shared_ptr<const urdf::Joint> jnt_cont2 = robot.getJoint("continuous2");
  boost::shared_ptr<const urdf::Joint> jnt_rev  = robot.getJoint("revolute");
#endif
  ASSERT_TRUE(jnt_cont1 != NULL);
  ASSERT_TRUE(jnt_cont2 != NULL);
  ASSERT_TRUE(jnt_rev != NULL);

  // test cont1
  pr2_mechanism_model::JointState js;
  pr2_hardware_interface::Actuator as;

  js.joint_ = jnt_cont1;
  js.position_ = 0.01;
  pr2_mechanism_model::JointCalibrationSimulator jnt_sim_1;
  jnt_sim_1.simulateJointCalibration(&js,&as);
  ASSERT_FALSE(as.state_.calibration_reading_);
  ASSERT_FALSE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);

  js.position_ = 1.0;
  as.state_.position_ = 10;
  jnt_sim_1.simulateJointCalibration(&js,&as);
  ASSERT_FALSE(as.state_.calibration_reading_);
  ASSERT_FALSE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);

  js.position_ = 1.6;
  as.state_.position_ = 16;
  jnt_sim_1.simulateJointCalibration(&js,&as);
  ASSERT_TRUE(as.state_.calibration_reading_);
  ASSERT_TRUE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);
  ASSERT_EQ(as.state_.last_calibration_rising_edge_, 10);

  js.position_ = 1.5001;
  as.state_.position_ = 150;
  jnt_sim_1.simulateJointCalibration(&js,&as);
  js.position_ = 1.4;
  as.state_.position_ = 14;
  jnt_sim_1.simulateJointCalibration(&js,&as);
  ASSERT_FALSE(as.state_.calibration_reading_);
  ASSERT_TRUE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);
  ASSERT_EQ(as.state_.last_calibration_rising_edge_, 150);

  js.position_ = 0.01;
  as.state_.position_ = 999;
  jnt_sim_1.simulateJointCalibration(&js,&as);
  js.position_ = -0.1;
  as.state_.position_ = -1;
  jnt_sim_1.simulateJointCalibration(&js,&as);
  ASSERT_TRUE(as.state_.calibration_reading_);
  ASSERT_TRUE(as.state_.calibration_rising_edge_valid_);
  ASSERT_TRUE(as.state_.calibration_falling_edge_valid_);
  ASSERT_EQ(as.state_.last_calibration_falling_edge_, 999);

  // test cont2
  as.state_.calibration_rising_edge_valid_ = false;
  as.state_.calibration_falling_edge_valid_ = false;
  js.joint_ = jnt_cont2;
  js.position_ = 0.1;
  pr2_mechanism_model::JointCalibrationSimulator jnt_sim_2;
  jnt_sim_2.simulateJointCalibration(&js,&as);
  ASSERT_TRUE(as.state_.calibration_reading_);
  ASSERT_FALSE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);

  js.position_ = 1.0;
  jnt_sim_2.simulateJointCalibration(&js,&as);
  ASSERT_TRUE(as.state_.calibration_reading_);
  ASSERT_FALSE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);

  js.position_ = 1.4;
  as.state_.position_ = 15;
  jnt_sim_2.simulateJointCalibration(&js,&as);
  js.position_ = 1.6;
  as.state_.position_ = 16;
  jnt_sim_2.simulateJointCalibration(&js,&as);
  ASSERT_FALSE(as.state_.calibration_reading_);
  ASSERT_FALSE(as.state_.calibration_rising_edge_valid_);
  ASSERT_TRUE(as.state_.calibration_falling_edge_valid_);
  ASSERT_EQ(as.state_.last_calibration_falling_edge_, 15);

  js.position_ = 1.6;
  as.state_.position_ = 15;
  jnt_sim_2.simulateJointCalibration(&js,&as);
  js.position_ = 1.4;
  as.state_.position_ = 14;
  jnt_sim_2.simulateJointCalibration(&js,&as);
  ASSERT_TRUE(as.state_.calibration_reading_);
  ASSERT_FALSE(as.state_.calibration_rising_edge_valid_);
  ASSERT_TRUE(as.state_.calibration_falling_edge_valid_);
  ASSERT_EQ(as.state_.last_calibration_falling_edge_, 15);  

  js.position_ = -0.1;
  as.state_.position_ = 0;
  jnt_sim_2.simulateJointCalibration(&js,&as);
  js.position_ = 0.1;
  as.state_.position_ = -1;
  jnt_sim_2.simulateJointCalibration(&js,&as);
  ASSERT_TRUE(as.state_.calibration_reading_);
  ASSERT_TRUE(as.state_.calibration_rising_edge_valid_);
  ASSERT_TRUE(as.state_.calibration_falling_edge_valid_);
  ASSERT_EQ(as.state_.last_calibration_rising_edge_, 0);  

  // test rev
  as.state_.calibration_rising_edge_valid_ = false;
  as.state_.calibration_falling_edge_valid_ = false;
  js.joint_ = jnt_rev;
  js.position_ = 0.01;
  pr2_mechanism_model::JointCalibrationSimulator jnt_sim_rev;
  jnt_sim_rev.simulateJointCalibration(&js,&as);
  ASSERT_FALSE(as.state_.calibration_reading_);
  ASSERT_FALSE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);

  js.position_ = 0.04;
  jnt_sim_rev.simulateJointCalibration(&js,&as);
  ASSERT_FALSE(as.state_.calibration_reading_);
  ASSERT_FALSE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);

  js.position_ = 0.5;
  as.state_.position_ = 5;
  jnt_sim_rev.simulateJointCalibration(&js,&as);
  js.position_ = 1.6;
  as.state_.position_ = 16;
  jnt_sim_rev.simulateJointCalibration(&js,&as);
  ASSERT_TRUE(as.state_.calibration_reading_);
  ASSERT_TRUE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);
  ASSERT_EQ(as.state_.last_calibration_rising_edge_, 5);

  js.position_ = 10.0;
  as.state_.position_ = 100.0;
  jnt_sim_rev.simulateJointCalibration(&js,&as);
  js.position_ = -10.0;
  as.state_.position_ = -100;
  jnt_sim_rev.simulateJointCalibration(&js,&as);
  ASSERT_FALSE(as.state_.calibration_reading_);
  ASSERT_TRUE(as.state_.calibration_rising_edge_valid_);
  ASSERT_FALSE(as.state_.calibration_falling_edge_valid_);


  SUCCEED();
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_joint_calibration_simulator");
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
