#include "test_controller.h"
#include <pluginlib/class_list_macros.h>

using namespace my_controller_ns;


/// Controller initialization in non-realtime
bool MyControllerClass::init(pr2_mechanism_model::RobotState *robot,
                             ros::NodeHandle &n)
{
  // get joint name
  std::string joint_name;
  if (!n.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  // get pointer to joint state
  joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("MyController could not find joint named '%s'",
              joint_name.c_str());
    return false;
  }

  // advertise service 
  srv_ = n.advertiseService("test",
                            &MyControllerClass::serviceCallback, this);


  // copy robot pointer so we can access time
  robot_ = robot;

  return true;
}/// Controller startup in realtime
void MyControllerClass::starting()
{
  init_pos_ = joint_state_->position_;
  time_of_last_cycle_ = robot_->getTime();
}


/// Controller update loop in realtime
void MyControllerClass::update()
{
  ros::Time time_of_last_cycle_ = robot_->getTime();
  double tmp;
  tmp = joint_state_->position_;
  tmp = joint_state_->velocity_;
  tmp = joint_state_->measured_effort_;
  joint_state_->commanded_effort_ = tmp;
}


/// Controller stopping in realtime
void MyControllerClass::stopping()
{}


/// Service call 
bool MyControllerClass::serviceCallback(pr2_mechanism_msgs::SwitchController::Request& req,
                                        pr2_mechanism_msgs::SwitchController::Response& resp)
{
  return true;
}

/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(TestController,
                         my_controller_ns::MyControllerClass,
                         pr2_controller_interface::Controller)
