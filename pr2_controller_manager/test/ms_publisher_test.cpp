///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include "ros/node.h"
#include "pr2_mechanism_msgs/MechanismState.h"

void publish(ros::Node *node)
{
  pr2_mechanism_msgs::MechanismState mechanism_state;

  mechanism_state.set_joint_states_size(250);
  mechanism_state.set_actuator_states_size(260);

  if (1)
  {
    for (unsigned int i = 0; i < mechanism_state.get_joint_states_size(); ++i)
    {
      pr2_mechanism_msgs::JointState *out = mechanism_state.joint_states + i;
      out->name = "jointstate";
      out->position = 1.0;
      out->velocity = 1.0;
      out->applied_effort = 1.0;
      out->commanded_effort = 1.0;
    }

    for (unsigned int i = 0; i < mechanism_state.get_actuator_states_size(); ++i)
    {
      pr2_mechanism_msgs::ActuatorState *out = mechanism_state.actuator_states + i;
      out->name = "actuatorstate";
      out->encoder_count = i;
      out->position = 1.0;
      out->timestamp = 1.0;
      out->encoder_velocity = 1.0;
      out->velocity = 1.0;
      out->calibration_reading = 0;
      out->last_calibration_rising_edge = i+2;
      out->last_calibration_falling_edge = i+1;
      out->is_enabled = 1;
      out->run_stop_hit = 1;
      out->last_requested_effort = 1.0;
      out->last_commanded_effort = 1.0;
      out->last_measured_effort = 1.0;
      out->motor_voltage = 1.0;
      out->num_encoder_errors = 0;
    }
    mechanism_state.time = 3.14159;

    node->publish("mechanism_state", mechanism_state);
  }

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv);
  ros::Node *node = new ros::Node("pr2_mechanism_control");

  node->advertise<pr2_mechanism_msgs::MechanismState>("mechanism_state");
  while (1) {
    std::cout << "publish" << std::endl;
    publish(node);
    usleep(100);
  }

  delete node;
  return 0;
}
