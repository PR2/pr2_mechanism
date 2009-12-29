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

#include <pr2_controller_manager/controller_manager.h>
#include <pr2_hardware_interface/hardware_interface.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "testRobot");
  ros::NodeHandle node;

  // create fake hardware interface with an actuator
  pr2_hardware_interface::HardwareInterface hw;
  hw.actuators_["head_tilt_motor"] = new pr2_hardware_interface::Actuator("head_tilt_motor");
  hw.actuators_["head_pan_motor"] = new pr2_hardware_interface::Actuator("head_pan_motor");
  pr2_controller_manager::ControllerManager cm(&hw, node);

  // read robot description from parameter server
  std::string robot_description_string;
  TiXmlDocument robot_description_xml;
  if (node.getParam("robot_description", robot_description_string))
    robot_description_xml.Parse(robot_description_string.c_str());
  else
  {
    ROS_ERROR("Could not load the robot description from the parameter server");
    return -1;
  }
  TiXmlElement *robot_description_root = robot_description_xml.FirstChildElement("robot");
  if (!robot_description_root)
  {
    ROS_ERROR("Could not parse the robot description");
    return -1;
  }
  
  // Initialize controller manager from robot description
  if (!cm.initXml(robot_description_root)){
    ROS_ERROR("Could not initialize controller manager");
    return -1;
  }

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Rate rate(1000.0);
  while (ros::ok()){
    cm.update();
    rate.sleep();
  }

  spinner.stop();

  return 0;
}


