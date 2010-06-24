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
#include <cstdlib>

#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <sensor_msgs/JointState.h>
#include <pr2_mechanism_msgs/MechanismStatistics.h>
#include <diagnostic_msgs/DiagnosticArray.h>

static const unsigned int _failure = 0;
static const unsigned int _unloaded = 1;
static const unsigned int _stopped = 2;
static const unsigned int _running = 3;


int g_argc;
char** g_argv;

static bool bombardment_started_ = false;


class TestController : public testing::Test
{
public:
  ros::NodeHandle node_;
  ros::ServiceClient load_srv_, unload_srv_, switch_srv_, list_srv_;
  std::string callback1_name_, callback4_name_;
  unsigned int callback1_counter_, callback_js_counter_, callback_ms_counter_;
  std::vector<ros::Time> callback1_timing_;
  unsigned int joint_diagnostic_counter_, controller_diagnostic_counter_;
  double callback1_effort_;

  void callbackDiagnostic(const diagnostic_msgs::DiagnosticArrayConstPtr& msg)
  {
    if (!msg->status.empty()){
      bool found_joint = false;
      bool found_controller = false;
      for (unsigned int i=0; i<msg->status.size(); i++){
        if (!found_joint && msg->status[i].name.substr(0,5) == "Joint")
          joint_diagnostic_counter_++;
        if (!found_controller && msg->status[i].name.substr(0,10) == "Controller")
          controller_diagnostic_counter_++;
      }
    }
  }

  void callbackJs(const sensor_msgs::JointStateConstPtr& msg)
  {
    if (!msg->name.empty())
      callback_js_counter_++;
  }

  void callbackMs(const pr2_mechanism_msgs::MechanismStatisticsConstPtr& msg)
  {
    callback_ms_counter_++;
  }

  void callback1(const sensor_msgs::JointStateConstPtr& msg)
  {
    if (callback1_counter_ < 1000){
      callback1_timing_[callback1_counter_] = ros::Time::now();
    }
    callback1_name_ = msg->name[0];
    callback1_effort_ = msg->effort[0];
    callback1_counter_++;
  }

  void callback4(const sensor_msgs::JointStateConstPtr& msg)
  {
    callback4_name_ = msg->name[0];
  }

  bool loadController(const std::string& name)
  {

    pr2_mechanism_msgs::LoadController srv_msg;
    srv_msg.request.name = name;
    if (!load_srv_.call(srv_msg)) return false;
    return srv_msg.response.ok;
  }

  bool unloadController(const std::string& name)
  {

    pr2_mechanism_msgs::UnloadController srv_msg;
    srv_msg.request.name = name;
    if (!unload_srv_.call(srv_msg)) return false;
    return srv_msg.response.ok;
  }

  bool switchController(const std::vector<std::string>& start, const std::vector<std::string>& stop, int strictness)
  {

    pr2_mechanism_msgs::SwitchController srv_msg;
    srv_msg.request.start_controllers = start;
    srv_msg.request.stop_controllers = stop;
    srv_msg.request.strictness = strictness;
    if (!switch_srv_.call(srv_msg)) return false;
    return srv_msg.response.ok;
  }

  unsigned int nrControllers()
  {

    pr2_mechanism_msgs::ListControllers srv_msg;
    if (!list_srv_.call(srv_msg)) return 0;;
    return srv_msg.response.controllers.size();
  }

  unsigned int controllerState(const std::string& name)
  {

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

  std::string randomController()
  {
      unsigned int random = rand();
      random = random % 10;
      std::stringstream s;
      s << random;
      return "controller" + s.str();
  }

  void randomBombardment(unsigned int times)
  {
    while (!bombardment_started_)
      ros::Duration(0.1).sleep();

    for (unsigned int i=0; i<times; i++){
      unsigned int random = rand();
      random = random % 4;
      switch (random){
      case 0:{
        loadController(randomController());
        break;
      }
      case 1:{
        unloadController(randomController());
        break;
      }
      case 2:{
        std::vector<std::string> start, stop;
        unsigned int start_size = rand() %10;
        unsigned int stop_size = rand() %10;
        for (unsigned int i=0; i<start_size; i++)
          start.push_back(randomController());
        for (unsigned int i=0; i<stop_size; i++)
          stop.push_back(randomController());
        if (rand() %1 == 0)
          switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::STRICT);
        else
          switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT);
        break;
      }
      case 3:{
        controllerState(randomController());
        break;
      }
      }
    }
  }
  

protected:
  /// constructor
  TestController()
    : callback1_timing_(1000)
  {
  }

  /// Destructor
  ~TestController()
  {
  }

  void SetUp()
  {
    ros::service::waitForService("pr2_controller_manager/load_controller");
    ros::service::waitForService("pr2_controller_manager/unload_controller");
    ros::service::waitForService("pr2_controller_manager/switch_controller");
    ros::service::waitForService("pr2_controller_manager/list_controllers");
    ros::service::waitForService("pr2_controller_manager/list_controllers");

    load_srv_ = node_.serviceClient<pr2_mechanism_msgs::LoadController>("pr2_controller_manager/load_controller");
    unload_srv_ = node_.serviceClient<pr2_mechanism_msgs::UnloadController>("pr2_controller_manager/unload_controller");
    switch_srv_ = node_.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");
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

  // this should be the controller state if spawner worked
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

  // this one is wrongly configured
  EXPECT_FALSE(loadController("controller9"));
  EXPECT_FALSE(loadController("controller11"));
  EXPECT_FALSE(loadController("controller12"));
  EXPECT_FALSE(loadController("controller13"));
  EXPECT_FALSE(loadController("controller14"));
  EXPECT_FALSE(loadController("controller15"));
  EXPECT_FALSE(loadController("controller16"));
  EXPECT_FALSE(loadController("controller17"));
  EXPECT_FALSE(loadController("controller18"));
  EXPECT_FALSE(loadController("controller19"));

  // this one is not configured
  EXPECT_FALSE(loadController("controller10"));

  // check end state
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _stopped);
  EXPECT_EQ(controllerState("controller3"), _running);
  EXPECT_EQ(controllerState("controller4"), _running);
  EXPECT_EQ(controllerState("controller5"), _stopped);
  EXPECT_EQ(controllerState("controller6"), _stopped);
  EXPECT_EQ(controllerState("controller7"), _stopped);
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

TEST_F(TestController, start_stop_strict)
{
  // check initial state
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _unloaded);
  EXPECT_EQ(controllerState("controller3"), _running);
  EXPECT_EQ(controllerState("controller4"), _running);
  EXPECT_EQ(controllerState("controller5"), _unloaded);
  EXPECT_EQ(controllerState("controller6"), _unloaded);
  EXPECT_EQ(controllerState("controller7"), _unloaded);
  EXPECT_EQ(controllerState("controller8"), _unloaded);
  EXPECT_EQ(controllerState("controller9"), _unloaded);

  // starting already started controller
  std::vector<std::string> start, stop;
  start.push_back("controller1");
  EXPECT_TRUE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::STRICT));
  EXPECT_EQ(controllerState("controller1"), _running);

  // starting unloaded controller
  start.push_back("controller2");
  EXPECT_FALSE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::STRICT));
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _unloaded);

  // starting one already stated, 1 stopped
  EXPECT_TRUE(loadController("controller2"));
  EXPECT_TRUE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::STRICT));
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _running);

  // start and stop same controller
  stop.push_back("controller2");
  EXPECT_TRUE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::STRICT));
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _running);

  // stop unloaded controller
  stop.push_back("controller5");
  EXPECT_FALSE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::STRICT));
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _running);
  EXPECT_EQ(controllerState("controller5"), _unloaded);

  // stop unloaded and running controller
  stop.push_back("controller4");
  EXPECT_FALSE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::STRICT));
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _running);
  EXPECT_EQ(controllerState("controller4"), _running);
  EXPECT_EQ(controllerState("controller5"), _unloaded);

  // stop running and stopped controller
  EXPECT_TRUE(loadController("controller5"));
  EXPECT_TRUE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::STRICT));
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _running);
  EXPECT_EQ(controllerState("controller4"), _stopped);
  EXPECT_EQ(controllerState("controller5"), _stopped);

  // stop 2 stopped controllers, and 1 running controller
  stop.push_back("controller3");
  EXPECT_TRUE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::STRICT));
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _running);
  EXPECT_EQ(controllerState("controller3"), _stopped);
  EXPECT_EQ(controllerState("controller4"), _stopped);
  EXPECT_EQ(controllerState("controller5"), _stopped);

  SUCCEED();
}


TEST_F(TestController, start_stop_best_effort)
{
  // check initial state
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller2"), _running);
  EXPECT_EQ(controllerState("controller3"), _stopped);
  EXPECT_EQ(controllerState("controller4"), _stopped);
  EXPECT_EQ(controllerState("controller5"), _stopped);
  EXPECT_EQ(controllerState("controller6"), _unloaded);
  EXPECT_EQ(controllerState("controller7"), _unloaded);
  EXPECT_EQ(controllerState("controller8"), _unloaded);
  EXPECT_EQ(controllerState("controller9"), _unloaded);

  // starting already started controller
  std::vector<std::string> start, stop;
  start.push_back("controller1");
  EXPECT_TRUE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT));
  EXPECT_EQ(controllerState("controller1"), _running);

  // starting unloaded, started and stopped controller
  start.push_back("controller3");
  start.push_back("controller6");
  EXPECT_TRUE(switchController(start, stop, pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT));
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller3"), _running);
  EXPECT_EQ(controllerState("controller6"), _unloaded);

  SUCCEED();
}



TEST_F(TestController, service_and_realtime_publisher)
{
  EXPECT_EQ(controllerState("controller1"), _running);
  EXPECT_EQ(controllerState("controller4"), _stopped);

  callback1_timing_.resize(1000);
  callback1_counter_ = 0;

  // connect to topic
  ros::Subscriber sub1 = node_.subscribe<sensor_msgs::JointState>("controller1/my_topic", 100, 
                                                                  boost::bind(&TestController_service_and_realtime_publisher_Test::callback1, this, _1));
  ros::Subscriber sub4 = node_.subscribe<sensor_msgs::JointState>("controller4/my_topic", 100, 
                                                                  boost::bind(&TestController_service_and_realtime_publisher_Test::callback4, this, _1));

  std::string not_started = "not_started";
  callback1_name_ = not_started;
  callback4_name_ = not_started;

  ros::Time start = ros::Time::now();
  ros::Duration timeout(5.0);
  while (callback1_name_ == "not_started" && ros::Time::now() - start < timeout)
    ros::Duration(0.1).sleep();
  ros::Duration(1.0).sleep(); // avoid problem with simultanious access to callback1_name_
  EXPECT_EQ(callback1_name_, "");
  EXPECT_EQ(callback4_name_, not_started);

  // check for effort limits
  for (unsigned int i=0; i<1000; i++){
    EXPECT_LE(callback1_effort_, 0.0);
    ros::Duration(0.01).sleep();
  }

  std::string test_message = "Hoe gaat het met Wim?";
  ros::ServiceClient srv_client1 = node_.serviceClient<pr2_mechanism_msgs::LoadController>("controller1/my_service");
  ros::ServiceClient srv_client4 = node_.serviceClient<pr2_mechanism_msgs::LoadController>("controller1/my_service");
  pr2_mechanism_msgs::LoadController srv_msg;
  srv_msg.request.name = test_message;
  EXPECT_TRUE(srv_client1.call(srv_msg));
  EXPECT_TRUE(srv_client4.call(srv_msg));

  ros::Duration(1.0).sleep();
  EXPECT_EQ(callback1_name_, test_message);
  EXPECT_EQ(callback4_name_, not_started);

  sub1.shutdown();
  sub4.shutdown();

  SUCCEED();
}


TEST_F(TestController, publisher_hz)
{
  EXPECT_EQ(controllerState("controller1"), _running);

  // connect to topic
  ros::Subscriber sub1 = node_.subscribe<sensor_msgs::JointState>("controller1/my_topic", 100, 
                                                                  boost::bind(&TestController_publisher_hz_Test::callback1, this, _1));

  callback1_counter_ = 0;
  ros::Time start = ros::Time::now();
  ros::Duration timeout(5.0);
  while (callback1_counter_ == 0 && ros::Time::now() - start < timeout)
    ros::Duration(0.01).sleep();
  callback1_counter_ = 0;
  ros::Duration(5.0).sleep();
  // gathering debugging info
  if (callback1_counter_ < 450){
    unsigned int nr = callback1_counter_;
    ROS_ERROR("Only received %u callbacks between start time %f and end time %f", nr, start.toSec(), ros::Time::now().toSec());
    for (unsigned int i=0; i<nr-1; i++){
      ROS_ERROR("  - %u:  time %f,  delta time %f", i, callback1_timing_[i].toSec(), (callback1_timing_[i+1]-callback1_timing_[i]).toSec());
    }
  }  
  EXPECT_NEAR(callback1_counter_, 500, 50);

  sub1.shutdown();
  SUCCEED();
}


TEST_F(TestController, manager_hz)
{
  // connect to topic
  ros::Subscriber sub_js = node_.subscribe<sensor_msgs::JointState>("joint_states", 100, 
                                                                    boost::bind(&TestController_manager_hz_Test::callbackJs, this, _1));
  callback_js_counter_ = 0;
  ros::Time start = ros::Time::now();
  ros::Duration timeout(5.0);
  while (callback_js_counter_ == 0 && ros::Time::now() - start < timeout)
    ros::Duration(0.1).sleep();
  callback_js_counter_ = 0;
  ros::Duration(5.0).sleep();
  EXPECT_NEAR(callback_js_counter_, 1000, 40);
  sub_js.shutdown();


  ros::Subscriber sub_ms = node_.subscribe<pr2_mechanism_msgs::MechanismStatistics>("mechanism_statistics", 100, 
                                                                                    boost::bind(&TestController_manager_hz_Test::callbackMs, this, _1));
  callback_ms_counter_ = 0;
  start = ros::Time::now();
  while (callback_ms_counter_ == 0 && ros::Time::now() - start < timeout)
    ros::Duration(0.1).sleep();
  callback_ms_counter_ = 0;
  ros::Duration(5.0).sleep();
  EXPECT_NEAR(callback_ms_counter_, 10, 2);
  sub_ms.shutdown();

  SUCCEED();
}


TEST_F(TestController, diagnostic_hz)
{
  // connect to topic
  ros::Subscriber sub_diagnostics = node_.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100, 
                                                                                      boost::bind(&TestController_manager_hz_Test::callbackDiagnostic, this, _1));
  joint_diagnostic_counter_ = 0;
  controller_diagnostic_counter_ = 0;
  ros::Duration(5.0).sleep();
  EXPECT_GT(joint_diagnostic_counter_, (unsigned int)1);
  EXPECT_GT(controller_diagnostic_counter_, (unsigned int)1);
  sub_diagnostics.shutdown();

  SUCCEED();
}



TEST_F(TestController, singlethread_bombardment)
{
  bombardment_started_ = true;
  randomBombardment(1000);
  bombardment_started_ = false;
  SUCCEED();
}

TEST_F(TestController, multithread_bombardment)
{
  boost::thread bomb1(boost::bind(&TestController_multithread_bombardment_Test::randomBombardment, this, 500));
  boost::thread bomb2(boost::bind(&TestController_multithread_bombardment_Test::randomBombardment, this, 500));
  boost::thread bomb3(boost::bind(&TestController_multithread_bombardment_Test::randomBombardment, this, 500));
  boost::thread bomb4(boost::bind(&TestController_multithread_bombardment_Test::randomBombardment, this, 500));
  bombardment_started_ = true;
  bomb1.join();
  bomb2.join();
  bomb3.join();
  bomb4.join();

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
