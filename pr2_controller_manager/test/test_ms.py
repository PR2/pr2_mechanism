#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: test_embed_msg.py 1986 2008-08-26 23:57:56Z sfkwc $

## Integration test for empty services to test serializers
## and transport

PKG = 'mechanism_control'
NAME = 'test_ms'
PUBTOPIC = "chatter"
LPNODE = 'listenerpublisher'
SUBTOPIC = 'listenerpublisher'

import roslib; roslib.load_manifest(PKG)

import sys, time
import unittest

# ad-hoc import here as we don't want to change the manifest for test dependencies
roslib.load_manifest('rostest')
import rospy, rostest
from mechanism_control.msg import MechanismState, ActuatorState, JointState

MSG = MechanismState

TIMEOUT = 10.0 #seconds

class TestMechanismState(unittest.TestCase):

    def setUp(self):
        self.callback_data = None
        
    def _test_ms_callback(self, data):
        print "GOT CALLBACK DATA"
        self.callback_data = data
    
    def test_ms_msg(self):
        self.assert_(self.callback_data is None, "invalid test fixture")

        # wait at most 5 seconds for listenerpublisher to be registered
        timeout_t = time.time() + 5.0
        while not rostest.is_subscriber(
            rospy.resolve_name(PUBTOPIC),
            rospy.resolve_name(LPNODE)) and time.time() < timeout_t:
            time.sleep(0.1)

        self.assert_(rostest.is_subscriber(
            rospy.resolve_name(PUBTOPIC),
            rospy.resolve_name(LPNODE)), "%s is not up"%LPNODE)
        
        print "Publishing to ", PUBTOPIC
        pub = rospy.Publisher(PUBTOPIC, MSG)
        print "Subscribing to ", SUBTOPIC
        rospy.Subscriber(SUBTOPIC, MSG, self._test_ms_callback) 

        # publish about 10 messages for fun
        import random
        val = random.randint(0, 109812312)
        msg = "hi [%s]"%val
        header = None
        for i in xrange(0, 10):
            # The test message could be better in terms of the values
            # it assigns to leaf fields, but the main focus is trying
            # to dig up edge conditions in the embeds, especially with
            # respect to arrays and embeds.
            actuator_states = [
                ActuatorState('name0', 0,
                              0.0, 0.0, 0.0, 0.0,
                              0, 0, 0, 0, 0,
                              0.0, 0.0, 0.0, 0.0, 0),
                ActuatorState('name1', 1,
                              1.0, 1.0, 1.0, 1.0,
                              1, 1, 1, 1, 1,
                              1.0, 1.0, 1.0, 1.0, 1),
                ]
            joint_states = [
                JointState('name0', 0.0, 0.0, 0.0, 0.0),
                JointState('name1', 1.0, 1.0, 1.0, 1.0),
                ]
            pub.publish(
                MSG(header, 2.0, 
                    actuator_states,
                    joint_states)
                )
            time.sleep(0.1)

        # listenerpublisher is supposed to repeat our messages back onto /listenerpublisher,
        # make sure we got it
        self.assert_(self.callback_data is not None, "no callback data from listenerpublisher")
        print "Got ", self.callback_data.time
        errorstr = "callback msg field [%s] from listenerpublisher does not match"
        self.assertEquals(2.0, self.callback_data.time,
                          errorstr%"time")
        self.assertEquals(2, len(self.callback_data.joint_states))
        self.assertEquals(2, len(self.callback_data.actuator_states))

        self.assertEquals("name0", self.callback_data.joint_states[0].name)
        self.assertEquals("name0", self.callback_data.actuator_states[0].name)
        self.assertEquals("name1", self.callback_data.joint_states[1].name)
        self.assertEquals("name1", self.callback_data.actuator_states[1].name)

        self.assertEquals(0, self.callback_data.actuator_states[0].encoder_count)
        self.assertEquals(0, self.callback_data.actuator_states[0].calibration_reading)        
        self.assertEquals(0, self.callback_data.actuator_states[0].last_calibration_rising_edge)
        self.assertEquals(0, self.callback_data.actuator_states[0].num_encoder_errors)
        
        self.assertEquals(1, self.callback_data.actuator_states[1].encoder_count)
        self.assertEquals(1, self.callback_data.actuator_states[1].calibration_reading)        
        self.assertEquals(1, self.callback_data.actuator_states[1].last_calibration_rising_edge)
        self.assertEquals(1, self.callback_data.actuator_states[1].num_encoder_errors)        
        
if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.run(PKG, NAME, TestMechanismState, sys.argv)
