#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <ros/ros.h>
#include <pr2_mechanism_msgs/SwitchController.h>


namespace my_controller_ns{

class MyControllerClass: public pr2_controller_interface::Controller
{
private:
  bool serviceCallback(pr2_mechanism_msgs::SwitchController::Request& req,
                       pr2_mechanism_msgs::SwitchController::Response& resp);

  pr2_mechanism_model::JointState* joint_state_;
  double init_pos_;
  double amplitude_;
  ros::ServiceServer srv_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time time_of_last_cycle_;

public:
  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();
};
}
