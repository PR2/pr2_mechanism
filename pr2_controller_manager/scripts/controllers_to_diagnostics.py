#! /usr/bin/python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('pr2_controller_manager')
import rospy

from pr2_mechanism_msgs.msg import MechanismStatistics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

def controller_to_diag(cs):
    d = DiagnosticStatus()
    d.name = 'Controllers'
    max_time = rospy.Duration()
    sum_time = rospy.Duration()
    if len(cs) == 0:
        d.values.append(KeyValue('No controllers loaded',''))
    else:
        for c in cs:
            if (c.running):
                state = 'Running'
            else:
                state = 'Stopped'
            state += '   (Avg time '+str(int(c.mean_time.to_sec()*1e6))+' usec)'
            sum_time = sum_time + c.mean_time
            state += '   (Max time '+str(int(c.max_time.to_sec()*1e6))+' usec)'
            if (max_time < c.max_time):
                max_time = c.max_time
            state += '   (Variance time '+str(int(c.variance_time.to_sec()*1e6))+')'
            state += '   (Uses '+str(int(c.mean_time.to_sec()/0.00001))+'% of cycle time)'
            d.values.append(KeyValue(c.name, state))
    if (max_time.to_sec() > 0.001 and not use_sim_time):
        d.level = 2
        d.message = 'One or more controllers used more than 1 ms to compute a single cycle'
    elif (sum_time.to_sec() > 0.001 and not use_sim_time):
        d.level = 2
        d.message = 'All controllers combined on average used more than 1 ms to compute a single cycle'
    else:
        d.level = 0
        d.message = 'OK'
    return d


rospy.init_node('controller_to_diagnostics')
use_sim_time = rospy.get_param('use_sim_time', False)
pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray)

last_publish_time = rospy.Time(0.0)
def state_cb(msg):
    global last_publish_time
    now = rospy.get_rostime()
    if (now - last_publish_time).to_sec() > 1.0:
        d = DiagnosticArray()
        d.header.stamp = msg.header.stamp
        d.status = [controller_to_diag(msg.controller_statistics)]
        pub_diag.publish(d)
        last_publish_time = now

rospy.Subscriber('mechanism_statistics', MechanismStatistics, state_cb)
rospy.spin()
