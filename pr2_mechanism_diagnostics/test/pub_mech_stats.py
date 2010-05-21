#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

PKG = 'pr2_mechanism_diagnostics'
import roslib; roslib.load_manifest(PKG)

import rospy
from pr2_mechanism_msgs.msg import MechanismStatistics, ControllerStatistics, JointStatistics

import random


RATE = 1.0

def joint_stat(name, calibrated = True, nan = False):
    jnt_st = JointStatistics()
    jnt_st.name = name
    jnt_st.position = random.uniform(-1, 1)
    if nan:
        jnt_st.position = float('NaN')
    jnt_st.is_calibrated = calibrated
    jnt_st.timestamp = rospy.get_rostime()    

    return jnt_st

def ctrl_stat(name, running = True, overrun = False):
    ctrl_st = ControllerStatistics()
    ctrl_st.name = name
    ctrl_st.timestamp = rospy.get_rostime()
    ctrl_st.running = running
    if overrun:
        ctrl_st.time_last_control_loop_overrun = rospy.get_rostime() - rospy.Duration(5)
        ctrl_st.num_control_loop_overruns = 10

    return ctrl_st

if __name__ == '__main__':
    rospy.init_node('mech_stat_pub')

    pub_cal = rospy.get_param("mech_diag/cal", True)
    pub_nan = rospy.get_param("mech_diag/nan", False)
    
    pub_running = rospy.get_param("mech_diag/running", True)
    pub_overrun = rospy.get_param("mech_diag/overrun", False)

    mech_st = MechanismStatistics()
    mech_st.joint_statistics = [ joint_stat('my_joint', pub_cal, pub_nan) ]
    mech_st.controller_statistics = [ ctrl_stat('my_controller', pub_running, pub_overrun) ]

    pub_mech_stats = rospy.Publisher('mechanism_statistics', MechanismStatistics)
    my_rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        pub_mech_stats.publish(mech_st)
        my_rate.sleep()


