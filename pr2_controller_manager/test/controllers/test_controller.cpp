#include "test_controller.h"
#include <pluginlib/class_list_macros.h>
#include <boost/thread/condition.hpp>

using namespace my_controller_ns;


/// Controller initialization in non-realtime
bool MyControllerClass::init(pr2_mechanism_model::RobotState *robot,
                             ros::NodeHandle &n)
{
  // copy robot pointer so we can access time
  robot_ = robot;


  // get joint
  // ----------
  std::string joint_name;
  if (!n.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint name given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("MyController could not find joint named '%s'",
              joint_name.c_str());
    return false;
  }
  if (!joint_state_->joint_->limits)
  {
    ROS_ERROR("MyController could not find limits of joint '%s'",
              joint_name.c_str());
    return false;
  }
  max_effort_ = joint_state_->joint_->limits->effort;



  // get chain
  // ----------
  std::string root_name, tip_name;
  if (!n.getParam("root_name", root_name))
  {
    ROS_ERROR("No root name given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("tip_name", tip_name))
  {
    ROS_ERROR("No tip name given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  if (!chain_.init(robot, root_name, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // advertise topic
  // ----------
  std::string topic_name;
  if (!n.getParam("topic_name", topic_name))
  {
    ROS_ERROR("No topic name given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(n, topic_name, 100));
  pub_->msg_.name.resize(1);
  pub_->msg_.name[0] = "";
  pub_->msg_.effort.resize(1);
  pub_->msg_.effort[0] = 0.0;


  // advertise service 
  // ----------
  std::string service_name;
  if (!n.getParam("service_name", service_name))
  {
    ROS_ERROR("No service name given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  srv_ = n.advertiseService(service_name, &MyControllerClass::serviceCallback, this);
  return true;
}

/// Controller startup in realtime
void MyControllerClass::starting()
{
  counter_ = 0;
  time_of_last_cycle_ = robot_->getTime();
}


/// Controller update loop in realtime
void MyControllerClass::update()
{
  counter_++;
  if (counter_ > 10 && pub_->trylock()){
    counter_ = 0;
    pub_->msg_.effort[0] = fabs(joint_state_->commanded_effort_) - max_effort_; // this should never be greater than zero
    pub_->unlockAndPublish();
  }

  ros::Time time_of_last_cycle_ = robot_->getTime();

  double tmp;
  tmp = joint_state_->position_;
  tmp = joint_state_->velocity_;
  tmp = joint_state_->measured_effort_;
  if (joint_state_->commanded_effort_ > 0)
    joint_state_->commanded_effort_ = -10000.0;  // above max effort
  else
    joint_state_->commanded_effort_ = 10000.0;  // above max effort
}


/// Controller stopping in realtime
void MyControllerClass::stopping()
{}


/// Service call 
bool MyControllerClass::serviceCallback(pr2_mechanism_msgs::LoadController::Request& req,
                                        pr2_mechanism_msgs::LoadController::Response& resp)
{
  pub_->msg_.name[0] = req.name;
  return true;
}

/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyControllerClass,
                       pr2_controller_interface::Controller);
