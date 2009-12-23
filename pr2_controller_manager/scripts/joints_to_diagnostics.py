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

def joint_to_diag(js):
    d = DiagnosticStatus()
    d.level = 0
    d.message = 'OK'

    # Hack to stop gripper joints from being "uncalibrated"
    if not js.is_calibrated and js.name.find("float") < 0 and js.name.find("finger") < 0:
        d.level = 1
        d.message = 'Uncalibrated'
    # Comment out because this causes confusing warnings
    #elif js.violated_limits:
    #    d.level = 1
    #    d.message = 'Joint limits violated'
    d.name = "Joint (%s)" % js.name
    d.values = [
        KeyValue('Position', str(js.position)),
        KeyValue('Velocity', str(js.velocity)),
        KeyValue('Measured Effort', str(js.measured_effort)),
        KeyValue('Commanded Effort', str(js.commanded_effort)),
        KeyValue('Calibrated', str(js.is_calibrated)),
        KeyValue('Odometer', str(js.odometer)),
        KeyValue('Max Position', str(js.max_position)),
        KeyValue('Min Position', str(js.min_position)),
        KeyValue('Max Abs. Velocity', str(js.max_abs_velocity)),
        KeyValue('Max Abs. Effort', str(js.max_abs_effort)),
        KeyValue('Limits Hit', str(js.violated_limits))]
        
    
    return d

rospy.init_node('joints_to_diagnostics')
pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray)

last_publish_time = rospy.Time(0.0)
def state_cb(msg):
    global last_publish_time
    now = rospy.get_rostime()
    if (now - last_publish_time).to_sec() > 1.0:
        d = DiagnosticArray()
        d.header.stamp = msg.header.stamp
        d.status = [joint_to_diag(js) for js in msg.joint_statistics]
        pub_diag.publish(d)
        last_publish_time = now

rospy.Subscriber('mechanism_statistics', MechanismStatistics, state_cb)
rospy.spin()
