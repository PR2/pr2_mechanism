#!/usr/bin/env python
PKG = 'pr2_controller_manager'

import curses
import roslib
roslib.load_manifest(PKG)

import sys, traceback, logging, rospy
from mechanism_msgs.msg import MechanismStatistics

NAME = 'joint_listener'

global stdscr

def mycmp(a, b):
  return cmp(abs(a[2]), abs(b[2]))

def callback(data):
    #stdscr.clear()
    l = [(data.actuator_states[x].device_id, data.actuator_states[x].encoder_count, data.actuator_states[x].encoder_velocity, data.actuator_states[x].name) for x in xrange(len(data.actuator_states))]
    m = list(l)
    m.sort(mycmp)
    row = 0
    for a in l:
      stdscr.move(row, 0)
      stdscr.clrtoeol()
      if (a[2] == m[-1][2] and m[-1][2] != 0):
        stdscr.attrset(curses.A_BOLD)
      else:
        stdscr.attrset(curses.A_NORMAL)

      if a[3] != "":
        stdscr.addstr(row, 0, "Device: %02d (%25s), count: %7d, velocity: %f" % (a[0], a[3], a[1], a[2]))
      else:
        stdscr.addstr(row, 0, "Device: %02d, count: %7d, velocity: %f" % (a[0], a[1], a[2]))
      row += 1
    stdscr.refresh()


def listener_with_user_data():
    rospy.Subscriber("/mechanism_statistics", MechanismStatistics, callback)
    rospy.init_node(NAME, anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    stdscr = curses.initscr()
    try:
        listener_with_user_data()
    except KeyboardInterrupt, e:
        pass
    stdscr.keypad(0)
    curses.echo();
    curses.nocbreak()
    curses.endwin()
