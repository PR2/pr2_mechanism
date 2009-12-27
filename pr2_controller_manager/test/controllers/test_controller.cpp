#include "test_controller.h"
#include <pluginlib/class_list_macros.h>

using namespace my_controller_ns;


/// Controller initialization in non-realtime
bool MyControllerClass::init(pr2_mechanism_model::RobotState *robot,
                             ros::NodeHandle &n)
{
  std::string service_name, topic_name;
  if (!n.getParam("service_name", service_name))
  {
    ROS_ERROR("No service name given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("topic_name", topic_name))
  {
    ROS_ERROR("No topic name given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }

  // copy robot pointer so we can access time
  robot_ = robot;

  // advertise topic
  pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(n, topic_name, 100));
  pub_->msg_.name.resize(1);
  pub_->msg_.name[0] = "";

  // advertise service 
  srv_ = n.advertiseService(service_name, &MyControllerClass::serviceCallback, this);

  return true;
}

/// Controller startup in realtime
void MyControllerClass::starting()
{
  time_of_last_cycle_ = robot_->getTime();
}


/// Controller update loop in realtime
void MyControllerClass::update()
{
  ros::Time time_of_last_cycle_ = robot_->getTime();

  if (pub_->trylock()){
    pub_->unlockAndPublish();
  }
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
PLUGINLIB_REGISTER_CLASS(TestController,
                         my_controller_ns::MyControllerClass,
                         pr2_controller_interface::Controller)
