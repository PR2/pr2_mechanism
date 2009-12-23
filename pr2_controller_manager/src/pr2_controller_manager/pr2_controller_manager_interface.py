#! /usr/bin/env python
# Wrappers around the services provided by MechanismControlNode

import roslib; roslib.load_manifest('pr2_controller_manager')

import sys

import rospy
from pr2_mechanism_msgs.srv import *

def list_controller_types():
    rospy.wait_for_service('pr2_controller_manager/list_controller_types')
    s = rospy.ServiceProxy('pr2_controller_manager/list_controller_types', ListControllerTypes)
    resp = s.call(ListControllerTypesRequest())
    for t in resp.types:
        print t

def reload_libraries(force_kill):
    rospy.wait_for_service('pr2_controller_manager/reload_controller_libraries')
    s = rospy.ServiceProxy('pr2_controller_manager/reload_controller_libraries', ReloadControllerLibraries)
    resp = s.call(ReloadControllerLibrariesRequest(force_kill))
    if resp.ok:
        print "Successfully reloaded libraries"
        return True
    else:
        print "Failed to reload libraries. Do you still have controllers loaded?"
        return False

def list_controllers():
    rospy.wait_for_service('pr2_controller_manager/list_controllers')
    s = rospy.ServiceProxy('pr2_controller_manager/list_controllers', ListControllers)
    resp = s.call(ListControllersRequest())
    if len(resp.controllers) == 0:
        print "No controllers are loaded in mechanism control"
    else:
        for c, s in zip(resp.controllers, resp.state):
            print c, "(",s,")"

def load_controller(name):
    rospy.wait_for_service('pr2_controller_manager/load_controller')
    s = rospy.ServiceProxy('pr2_controller_manager/load_controller', LoadController)
    resp = s.call(LoadControllerRequest(name))
    if resp.ok:
        print "Loaded", name
        return True
    else:
        print "Error when loading", name
        return False

def unload_controller(name):
    rospy.wait_for_service('pr2_controller_manager/unload_controller')
    s = rospy.ServiceProxy('pr2_controller_manager/unload_controller', UnloadController)
    resp = s.call(UnloadControllerRequest(name))
    if resp.ok == 1:
        print "Unloaded %s successfully" % name
        return True
    else:
        print "Error when unloading", name
        return False

def start_controller(name):
    return start_stop_controllers([name], True)

def start_controllers(names):
    return start_stop_controllers(names, True)

def stop_controller(name):
    return start_stop_controllers([name], False)

def stop_controllers(names):
    return start_stop_controllers(name, False)

def start_stop_controllers(names, st):
    rospy.wait_for_service('pr2_controller_manager/switch_controller')
    s = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
    start = []
    stop = []
    strictness = SwitchControllerRequest.STRICT
    if st:
        start = names
    else:
        stop = names
    resp = s.call(SwitchControllerRequest(start, stop, strictness))
    if resp.ok == 1:
        if st:
            print "Started %s successfully" % names
        else:
            print "Stopped %s successfully" % names
        return True
    else:
        if st:
            print "Error when starting ", names
        else:
            print "Error when stopping ", names
        return False
