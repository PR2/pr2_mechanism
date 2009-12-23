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
# Revision $Id: listenerpublisher_embed.py 1986 2008-08-26 23:57:56Z sfkwc $

## Utility node for testing, configured for MechanismState

PKG = 'mechanism_control'
NAME = 'listenerpublisher'
IN  = "chatter"
OUT = "listenerpublisher" 

import roslib; roslib.load_manifest(PKG)

import sys, traceback, logging, rospy

from mechanism_control.msg import MechanismState

MSG = MechanismState

_publishing = False
_pub = None
def start_publishing():
    global _pub
    if _pub is not None:
        return
    print "registering onto %s"%OUT
    _pub = rospy.Publisher(OUT, MSG)
    
def callback(data):
    start_publishing()
    print "re-publishing"
    _pub.publish(data)
    
def listener_publisher():
    rospy.init_node(NAME)
    rospy.Subscriber(IN, MSG, callback)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        listener_publisher()
    except KeyboardInterrupt, e:
        pass
    print "exiting"

