#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

##\author Kevin Watts

##\brief Tests receipt of test monitor messages from life tests

from __future__ import with_statement

DURATION = 10

PKG = 'pr2_mechanism_diagnostics'
import roslib; roslib.load_manifest(PKG)

import unittest
import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser
import math

from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse
from pr2_mechanism_msgs.msg import MechanismStatistics, JointStatistics, ActuatorStatistics
from diagnostic_msgs.msg import DiagnosticArray

import threading

FLEX_JOINT = 'r_elbow_flex_joint'
ROLL_JOINT = 'r_forearm_roll_joint'
FLEX_MOTOR = 'r_elbow_flex_motor'
ROLL_MOTOR = 'r_forearm_roll_motor'

WRIST_FLEX_JOINT = 'r_wrist_flex_joint'
WRIST_ROLL_JOINT = 'r_wrist_roll_joint'
WRIST_FLEX_MOTOR = 'r_wrist_l_motor'
WRIST_ROLL_MOTOR = 'r_wrist_r_motor'

def check_status(msg, joint_name):
    for stat in msg.status:
        rospy.loginfo('Joint: %s. Status: %s' % (joint_name, stat.name))
        if stat.name.find(joint_name) == -1:
            continue
        if stat.name.startswith('Transmission'):
            return stat.level

    return -1

def mech_stats(flex_error = False, roll_error = False, start_time = 0.0):
    # Joint state is a sine, period 1s, Amplitude 2,
    trig_arg = rospy.get_time() - start_time
    
    sine = math.sin(trig_arg)
    cosine = math.cos(trig_arg)
    
    jnt_st = JointStatistics()
    jnt_st.name = FLEX_JOINT
    jnt_st.position = float(1 * sine) - 1.2
    jnt_st.is_calibrated = 1
    
    cont_st = JointStatistics()
    cont_st.name = ROLL_JOINT
    cont_st.position = 5 * float(0.5 * sine)
    cont_st.velocity = 2.5 * float(0.5 * cosine)
    cont_st.is_calibrated = 1
    
    cont_act_st = ActuatorStatistics()
    cont_act_st.name = ROLL_MOTOR
    cont_act_st.calibration_reading = False 
    wrapped_position = (cont_st.position % 6.28)
    if wrapped_position < 3.14:
        cont_act_st.calibration_reading = True
    if roll_error:
        cont_act_st.calibration_reading = not cont_act_st.calibration_reading

    act_st = ActuatorStatistics()
    act_st.name = FLEX_MOTOR
    act_st.calibration_reading = True
    if jnt_st.position > -1.1606:
        act_st.calibration_reading = False
    if flex_error:
        act_st.calibration_reading = not act_st.calibration_reading
        
    mech_st = MechanismStatistics()
    mech_st.actuator_statistics = [ act_st, cont_act_st ]
    mech_st.joint_statistics = [ jnt_st, cont_st ]
    mech_st.header.stamp = rospy.get_rostime()
    
    return mech_st

def wrist_mech_stats(flex_error = False, roll_error = False, start_time = 0.0):
    # Joint state is a sine wave, period 1s
    trig_arg = rospy.get_time() - start_time
    
    sine = math.sin(trig_arg)
    cosine = math.cos(trig_arg)
    
    jnt_st = JointStatistics()
    jnt_st.name = WRIST_FLEX_JOINT
    jnt_st.position = float(1 * sine) - 1.2
    jnt_st.is_calibrated = 1
    
    cont_st = JointStatistics()
    cont_st.name = WRIST_ROLL_JOINT
    cont_st.position = 5 * float(0.5 * sine)
    cont_st.velocity = 2.5 * float(0.5 * cosine)
    cont_st.is_calibrated = 1
    
    cont_act_st = ActuatorStatistics()
    cont_act_st.name = WRIST_ROLL_MOTOR
    cont_act_st.calibration_reading = False 
    wrapped_position = (cont_st.position % 6.28)
    if wrapped_position > 3.14159 + 1.5707 or wrapped_position < 1.5707:
        cont_act_st.calibration_reading = True
    if roll_error:
        cont_act_st.calibration_reading = not cont_act_st.calibration_reading

    act_st = ActuatorStatistics()
    act_st.name = WRIST_FLEX_MOTOR
    act_st.calibration_reading = True
    if jnt_st.position > -0.54105:
        act_st.calibration_reading = False
    if flex_error:
        act_st.calibration_reading = not act_st.calibration_reading
        
    mech_st = MechanismStatistics()
    mech_st.actuator_statistics = [ act_st, cont_act_st ]
    mech_st.joint_statistics = [ jnt_st, cont_st ]
    mech_st.header.stamp = rospy.get_rostime()
    
    return mech_st


class TransListenerErrorTest(unittest.TestCase):
    def __init__(self, *args):
        super(TransListenerErrorTest, self).__init__(*args)

        self._mutex = threading.Lock()
        rospy.init_node('test_trans_listener_error')
        self._ignore_time = 5 # Ignore values for about 5 seconds
        self._start_time = rospy.get_time()
        self._ok = True
        self._message = None
        self._level = 0

        self._roll_error = rospy.get_param("~roll_error", False)
        self._flex_error = rospy.get_param("~flex_error", False)
        self._roll_warn = rospy.get_param("~roll_warn", False)
        self._flex_warn = rospy.get_param("~flex_warn", False)

        self._wrist = rospy.get_param("~wrist", False)
        self._flex_joint = WRIST_FLEX_JOINT if self._wrist else FLEX_JOINT
        self._roll_joint = WRIST_ROLL_JOINT if self._wrist else ROLL_JOINT
        self._flex_motor = WRIST_FLEX_MOTOR if self._wrist else FLEX_MOTOR
        self._roll_motor = WRIST_ROLL_MOTOR if self._wrist else ROLL_MOTOR

        self._start_time = rospy.get_time()

        self._diag_msg = None
        self._trans_status = False
        self._trans_msg = None
        
        self._diag_sub  = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_cb)
        self.mech_pub   = rospy.Publisher('mechanism_statistics', MechanismStatistics)
        self._reset_srv = rospy.Service('pr2_etherCAT/reset_motors', Empty, self.on_reset)
        self._halt_srv  = rospy.Service('pr2_etherCAT/halt_motors', Empty, self.on_halt)
        self._trans_sub = rospy.Subscriber('pr2_mechanism_diagnostics/transmission_status', Bool, self._trans_cb)

    def _trans_cb(self, msg):
        with self._mutex:
            self._trans_msg = msg
            self._trans_status = msg.data

    def _diag_cb(self, msg):
        with self._mutex:
            self._diag_msg = msg

    def on_halt(self, srv):
        rospy.loginfo('Halt motors called')
        with self._mutex:
            self._ok = False
        return EmptyResponse()

    def on_reset(self, srv):
        rospy.loginfo('Reset motors called')
        with self._mutex:
            self._ok = True
        return EmptyResponse()

    def publish_mech_stats(self, flex_error = False, roll_error = False):
        if self._wrist:
            mech_st = wrist_mech_stats(flex_error, roll_error, self._start_time)
        else:
            mech_st = mech_stats(flex_error, roll_error, self._start_time)

        self.mech_pub.publish(mech_st)
    
    def test_monitor(self):
        while not rospy.is_shutdown():
            sleep(0.1)
            self.publish_mech_stats()
            if rospy.get_time() - self._start_time > DURATION:
                break
        rospy.loginfo('Sent some normal data')
        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")

            self.assert_(self._diag_msg, "No diagnostics message received")
            self.assert_(self._trans_msg, "Transmission status topic didn't publish")
            self.assert_(self._trans_status, "Transmission status topic reported error!")
            
            lvl = check_status(self._diag_msg, self._flex_joint)
            self.assert_(lvl  == 0, "Flex transmission wasn't OK. Level: %d" % lvl)

            lvl = check_status(self._diag_msg, self._roll_joint)
            self.assert_(lvl  == 0, "Roll transmission wasn't OK. Level: %d" % lvl)

        for i in range(0, 10): # 10 consecutive errors
            self.publish_mech_stats(self._flex_error, self._roll_error)
            sleep(0.1)

        # Publish one value if we have warnings
        self.publish_mech_stats(self._flex_warn, self._roll_warn)
        sleep(0.1)

        for i in range(0, 10): # Some OK values
            self.publish_mech_stats()
            sleep(0.1)
                
        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")
            
            if self._roll_error or self._flex_error:
                self.assert_(not self._ok, "Transmission listener didn't call halt motors!")
                self.assert_(not self._trans_status, "Transmission status topic didn't report error")
            else:
                self.assert_(self._ok, "Transmission listener called halt motors!")
                self.assert_(self._trans_status, "Transmission status topic reported error")


            flex_val = 2 if self._flex_error else 0
            lvl = check_status(self._diag_msg, self._flex_joint)
            self.assert_(lvl  == flex_val, "Flex transmission level incorrect. Level: %d. Expected: %d" % (lvl, flex_val))

            roll_val = 2 if self._roll_error else 0
            lvl = check_status(self._diag_msg, self._roll_joint)
            self.assert_(lvl  == roll_val, "Roll transmission level incorrect. Level: %d. Expected: %d" % (lvl, roll_val))

            


if __name__ == '__main__':
    print 'SYS ARGS:', sys.argv
    rostest.run(PKG, sys.argv[0], TransListenerErrorTest, sys.argv)
