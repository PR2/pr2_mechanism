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

from __future__ import with_statement
PKG = 'pr2_mechanism_diagnostics'
import roslib; roslib.load_manifest(PKG)

import rospy, rostest, unittest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import sys
import threading
from time import sleep

WAIT_TIME = 5

class TestMechDiag(unittest.TestCase):
    def __init__(self, *args):
        super(TestMechDiag, self).__init__(*args)

        self._mutex = threading.Lock()
        self._joints = {}
        self._controllers = {}
        
        self._start_time = rospy.get_time()

        # Calibrated, Nan for joints
        self._cal = rospy.get_param("mech_diag/cal", True)
        self._nan = rospy.get_param("mech_diag/nan", False)
        
        self._running = rospy.get_param("mech_diag/running", True)
        self._overrun = rospy.get_param("mech_diag/overrun", False)
        
        sub_diag = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_cb)

    def _diag_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                if stat.name.startswith('Joint'):
                    self._joints[stat.name] = stat
                if stat.name.startswith('Controller'):
                    # Ignore special "No controllers" status
                    if stat.name.find('No controllers') > 0:
                        continue

                    self._controllers[stat.name] = stat

    def test_mech_diag(self):
        while not rospy.is_shutdown() and (rospy.get_time() - self._start_time) < WAIT_TIME:
            sleep(0.5)

        self.assert_(not rospy.is_shutdown(), "Rospy shutdown")

        with self._mutex:
            if len(self._joints.items()) == 0:
                self.assert_(False, "No joint data in diagnostics")
            for key, val in self._joints.iteritems():
                if self._nan:
                    self.assert_(val.level == 2, "Joint %s was not ERROR, but was NaN. Level %d" % (val.name, val.level))
                elif self._cal:
                    self.assert_(val.level == 0, "Joint %s was not OK. Level %d" % (val.name, val.level))
                else:
                    self.assert_(val.level == 1, "Joint %s was not warning, but was uncalibrated. Level %d" % (val.name, val.level))

            if len(self._controllers.items()) == 0:
                self.assert_(False, "No controller data in diagnostics")

            for key, val in self._controllers.iteritems():
                if self._overrun:
                    self.assert_(val.level == 1, "Controller %s was not WARN, but was overrun. Level %d" % (val.name, val.level))
                else:
                    self.assert_(val.level == 0, "Controller %s was not OK. Level %d" % (val.name, val.level))
                    
if __name__ == '__main__':
    rospy.init_node('test_mech_diag_nominal')
    if True:
        rostest.run(PKG, sys.argv[0], TestMechDiag, sys.argv)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestMechDiag('test_mech_diag'))

        unittest.TextTestRunner(verbosity = 2).run(suite)
