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

def controller_to_diag(c):
    d = DiagnosticStatus()
    d.name = 'Controller ('+c.name+')'

    d.level = 0
    if (c.running):
        d.message = 'Running'
    else:
        d.message = 'Stopped'

    if (not use_sim_time and c.num_control_loop_overruns > 0):
        d.message += ' !!! Broke Realtime, used '+str(int(c.max_time.to_sec()*1e6))+' micro seconds in update loop'
        if c.time_last_control_loop_overrun + rospy.Duration(30.0) > rospy.Time.now():
            d.level = 1

    d.values.append(KeyValue('Avg Time in Update Loop (usec)',str(int(c.mean_time.to_sec()*1e6))))
    d.values.append(KeyValue('Max Time in update Loop (usec)',str(int(c.max_time.to_sec()*1e6))))
    d.values.append(KeyValue('Variance on Time in Update Loop',str(int(c.variance_time.to_sec()*1e6))))
    d.values.append(KeyValue('Percent of Cycle Time Used',str(int(c.mean_time.to_sec()/0.00001))))
    d.values.append(KeyValue('Number of Control Loop Overruns',str(int(c.num_control_loop_overruns))))
    d.values.append(KeyValue('Timestamp of Last Control Loop Overrun (sec)',str(int(c.time_last_control_loop_overrun.to_sec()))))
    return d

rospy.init_node('controller_to_diagnostics')
use_sim_time = rospy.get_param('use_sim_time', False)
pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray)

last_publish_time = rospy.Time(0.0)
def state_cb(msg):
    global last_publish_time
    now = rospy.get_rostime()
    if (now - last_publish_time).to_sec() > 1.0:
        if len(msg.controller_statistics) == 0:
            d = DiagnosticArray()
            d.header.stamp = msg.header.stamp
            ds = DiagnosticStatus()
            ds.name = "Controller: No controllers loaded in controller manager"
            d.status = [ds]
            pub_diag.publish(d)
        else:
            for c in msg.controller_statistics:
                d = DiagnosticArray()
                d.header.stamp = msg.header.stamp
                d.status = [controller_to_diag(c)]
                pub_diag.publish(d)
        last_publish_time = now

rospy.Subscriber('mechanism_statistics', MechanismStatistics, state_cb)
rospy.spin()
