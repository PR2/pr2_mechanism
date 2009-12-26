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
#include <boost/thread.hpp>

#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <pr2_mechanism_msgs/SwitchController.h>

static const unsigned int _failure = 0;
static const unsigned int _unloaded = 0;
static const unsigned int _stopped = 0;
static const unsigned int _running = 0;


int g_argc;
char** g_argv;


class TestController : public testing::Test
{
public:
  ros::NodeHandle node_;
  ros::ServiceClient load_srv_, unload_srv_, switch_srv_, list_srv_;

  bool loadController(const std::string& name)
  {
    ros::service::waitForService("pr2_controller_manager/load_controller");

    pr2_mechanism_msgs::LoadController srv_msg;
    srv_msg.request.name = name;
    if (!load_srv_.call(srv_msg)) return false;
    return srv_msg.response.ok;
  }

  bool unloadController(const std::string& name)
  {
    ros::service::waitForService("pr2_controller_manager/unload_controller");

    pr2_mechanism_msgs::UnloadController srv_msg;
    srv_msg.request.name = name;
    if (!unload_srv_.call(srv_msg)) return false;
    return srv_msg.response.ok;
  }

  unsigned int nrControllers()
  {
    ros::service::waitForService("pr2_controller_manager/list_controllers");

    pr2_mechanism_msgs::ListControllers srv_msg;
    if (!list_srv_.call(srv_msg)) return 0;;
    return srv_msg.response.controllers.size();
  }

  unsigned int controllerState(const std::string& name)
  {
    ros::service::waitForService("pr2_controller_manager/list_controllers");

    pr2_mechanism_msgs::ListControllers srv_msg;
    if (!list_srv_.call(srv_msg)) return _failure;
    for (unsigned int i=0; i<srv_msg.response.controllers.size(); i++){
      if (name == srv_msg.response.controllers[i]){
        if (srv_msg.response.state[i] == "running") return _running;
        else if (srv_msg.response.state[i] == "stopped") return _stopped;
        else return _failure;
      }
    }
    return _unloaded;
  }


protected:
  /// constructor
  TestController()
  {
  }

  /// Destructor
  ~TestController()
  {
  }

  void SetUp()
  {
    load_srv_ = node_.serviceClient<pr2_mechanism_msgs::LoadController>("pr2_controller_manager/load_controller");
    unload_srv_ = node_.serviceClient<pr2_mechanism_msgs::UnloadController>("pr2_controller_manager/unload_controller");
    list_srv_ = node_.serviceClient<pr2_mechanism_msgs::ListControllers>("pr2_controller_manager/list_controllers");
  }

  void TearDown()
  {
    load_srv_.shutdown();
  }
};




// test spawner
TEST_F(TestController, spawner)
{
  // wait until all controllers are loaded
  ros::Time start = ros::Time::now();
  ros::Duration timeout(60.0);
  while (nrControllers() != 6 && ros::Time::now() - start < timeout)
    ros::Duration(1.0).sleep();
  EXPECT_TRUE(ros::Time::now() - start < timeout);

  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _stopped);
  EXPECT_EQ(controllerState("controller3"), _running);
  EXPECT_EQ(controllerState("controller4"), _running);
  EXPECT_EQ(controllerState("controller5"), _stopped);
  EXPECT_EQ(controllerState("controller6"), _stopped);
  EXPECT_EQ(controllerState("controller7"), _unloaded);
  SUCCEED();
}


TEST_F(TestController, loading)
{
  // check initial state
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _stopped);
  EXPECT_EQ(controllerState("controller3"), _running);
  EXPECT_EQ(controllerState("controller4"), _running);
  EXPECT_EQ(controllerState("controller5"), _stopped);
  EXPECT_EQ(controllerState("controller6"), _stopped);
  EXPECT_EQ(controllerState("controller7"), _unloaded);
  EXPECT_EQ(controllerState("controller8"), _unloaded);
  EXPECT_EQ(controllerState("controller9"), _unloaded);

  // these are already loaded
  EXPECT_FALSE(loadController("controller1"));
  EXPECT_FALSE(loadController("controller2"));
  EXPECT_FALSE(loadController("controller3"));
  EXPECT_FALSE(loadController("controller4"));
  EXPECT_FALSE(loadController("controller5"));
  EXPECT_FALSE(loadController("controller6"));

  // this one is not loaded yet
  EXPECT_TRUE(loadController("controller7"));

  // check end state
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _stopped);
  EXPECT_EQ(controllerState("controller3"), _running);
  EXPECT_EQ(controllerState("controller4"), _running);
  EXPECT_EQ(controllerState("controller5"), _stopped);
  EXPECT_EQ(controllerState("controller6"), _stopped);
  EXPECT_EQ(controllerState("controller7"), _unloaded);
  EXPECT_EQ(controllerState("controller8"), _unloaded);
  EXPECT_EQ(controllerState("controller9"), _unloaded);

  SUCCEED();
}

TEST_F(TestController, unloading)
{
  // check initial state
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _stopped);
  EXPECT_EQ(controllerState("controller3"), _running);
  EXPECT_EQ(controllerState("controller4"), _running);
  EXPECT_EQ(controllerState("controller5"), _stopped);
  EXPECT_EQ(controllerState("controller6"), _stopped);
  EXPECT_EQ(controllerState("controller7"), _stopped);
  EXPECT_EQ(controllerState("controller8"), _unloaded);
  EXPECT_EQ(controllerState("controller9"), _unloaded);

  // these are running, so unloading should fail
  EXPECT_FALSE(unloadController("controller1"));
  EXPECT_FALSE(unloadController("controller3"));
  EXPECT_FALSE(unloadController("controller4"));

  // these are stopped, so unloading should succeed
  EXPECT_TRUE(unloadController("controller2"));
  EXPECT_TRUE(unloadController("controller5"));
  EXPECT_TRUE(unloadController("controller6"));
  EXPECT_TRUE(unloadController("controller7"));

  // this one is not loaded, so unloading should fail
  EXPECT_FALSE(unloadController("controller8"));

  // check end state
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _unloaded);
  EXPECT_EQ(controllerState("controller3"), _running);
  EXPECT_EQ(controllerState("controller4"), _running);
  EXPECT_EQ(controllerState("controller5"), _unloaded);
  EXPECT_EQ(controllerState("controller6"), _unloaded);
  EXPECT_EQ(controllerState("controller7"), _unloaded);
  EXPECT_EQ(controllerState("controller8"), _unloaded);
  EXPECT_EQ(controllerState("controller9"), _unloaded);

  SUCCEED();
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;

  ros::init(g_argc, g_argv, "testControllers");

  boost::thread spinner(boost::bind(&ros::spin));

  int res = RUN_ALL_TESTS();
  spinner.interrupt();
  spinner.join();

  return res;
}
