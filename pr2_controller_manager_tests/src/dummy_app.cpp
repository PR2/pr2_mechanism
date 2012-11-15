///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include <pr2_hardware/controller_manager.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <tinyxml.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DummyApp");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  pr2_hardware_interface::HardwareInterface hw;

  pr2_hardware::ControllerManager cm(&hw, nh);


  std::string urdf_string;
  if (!nh.getParam("robot_description", urdf_string))
  {
    ROS_ERROR("Could not find URDF. Exiting...");
    return -1;
  }

  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf_string.c_str()) && doc.Error())
  {
    ROS_ERROR("Could not load the gazebo controller manager plugin's configuration file: %s\n",
              urdf_string.c_str());
  }
  else
  {
    //doc.Print();
    //std::cout << *(doc.RootElement()) << std::endl;

    // Pulls out the list of actuators used in the robot configuration.
    struct GetActuators : public TiXmlVisitor
    {
      std::set<std::string> actuators;
      virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
      {
        if (elt.ValueStr() == std::string("actuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() == std::string("rightActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() == std::string("leftActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        return true;
      }
    } get_actuators;
    doc.RootElement()->Accept(&get_actuators);

    // Places the found actuators into the hardware interface.
    std::set<std::string>::iterator it;
    for (it = get_actuators.actuators.begin(); it != get_actuators.actuators.end(); ++it)
    {
      //std::cout << " adding actuator " << (*it) << std::endl;
      pr2_hardware_interface::Actuator* pr2_actuator = new pr2_hardware_interface::Actuator(*it);
      pr2_actuator->state_.is_enabled_ = true;
      hw.addActuator(pr2_actuator);
    }

    cm.initXml(doc.RootElement());
    for (unsigned int i = 0; i < cm.state_->joint_states_.size(); ++i)
      cm.state_->joint_states_[i].calibrated_ = true;
  }

  while (ros::ok())
  {
    ROS_INFO("loop");
    cm.update();
    ros::Duration(1.0).sleep();
  }
}

