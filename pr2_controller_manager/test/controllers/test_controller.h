#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/chain.h>
#include <ros/ros.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/scoped_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/chain.hpp>


namespace my_controller_ns{

class MyControllerClass: public pr2_controller_interface::Controller
{
private:
  bool serviceCallback(pr2_mechanism_msgs::LoadController::Request& req,
                       pr2_mechanism_msgs::LoadController::Response& resp);

  ros::ServiceServer srv_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > pub_;
  pr2_mechanism_model::RobotState *robot_;
  pr2_mechanism_model::JointState *joint_state_;
  pr2_mechanism_model::Chain chain_;
  KDL::Chain kdl_chain_;
  ros::Time time_of_last_cycle_;
  double max_effort_;
  unsigned int counter_;

public:
  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();
};
}
