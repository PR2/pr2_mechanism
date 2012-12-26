#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_controller_manager')
import rospy

def debugeval(text):
    print "About to eval: %s" % text
    return eval(text)

def debugexec(text):
    print "About to exec: %s" % text
    exec(text)

class ServiceMapper:
  def __init__(self, from_pkg, from_type,
                     to_pkg,   to_type,
                     from_topic, to_topic,
                     req_map, resp_map):

      self.req_map  = req_map
      self.resp_map = resp_map
      self.from_type = from_type
      self.to_type   = to_type
      self.from_pkg = from_pkg
      self.to_pkg   = to_pkg

      exec("import %s.srv" % self.to_pkg)
      exec("import %s.srv" % self.from_pkg)

      found_service = False
      while not found_service and not rospy.is_shutdown():
          try:
              rospy.wait_for_service(to_topic, timeout=1.0)
              found_service = True
          except:
              rospy.logwarn('Waiting for service [%s]' % to_topic)

      if rospy.is_shutdown():
          return

      self.client = rospy.ServiceProxy(to_topic, eval('%s.srv.%s' % (to_pkg,   to_type)))
      self.server = rospy.Service(from_topic,    eval('%s.srv.%s' % (from_pkg, from_type)), self.service_cb)

  def service_cb(self, from_req):
      exec("import %s.srv" % self.to_pkg)
      exec("import %s.srv" % self.from_pkg)

      to_req = eval("%s.srv.%sRequest()" % (self.to_pkg, self.to_type))
      for t,f in self.req_map.iteritems():
          exec( 'to_req.%s = from_req.%s' % (t,f) )

      to_resp = self.client.call(to_req)

      from_resp = eval("%s.srv.%sResponse()" % (self.from_pkg, self.from_type))
      for f,t in self.resp_map.iteritems():
          # dealing with a list
          if '[]' in t:
              exec('t_list = to_resp.%s'%(t.split('[]')[0]))
              t_item = t.split('[]')[1]
              for l in t_list:
                  exec( 'from_resp.%s = l.%s' %(f, t_item))

          # dealing with a regular object that we can just copy
          else:
              exec( 'from_resp.%s = to_resp.%s' % (f,t) )
      return from_resp


if __name__ == "__main__":
    rospy.init_node('pr2_controller_manager_adapter')

    load_controller = ServiceMapper( 'pr2_mechanism_msgs', 'LoadController',  'controller_manager_msgs', 'LoadController',
                                     'pr2_controller_manager/load_controller', 'controller_manager/load_controller',
                                     { 'name': 'name' },
                                     { 'ok':   'ok'   } )

    list_controllers = ServiceMapper( 'pr2_mechanism_msgs', 'ListControllers',  'controller_manager_msgs', 'ListControllers',
                                      'pr2_controller_manager/list_controllers', 'controller_manager/list_controllers',
                                      { },
                                      { 'name':   'controller[]/name',
                                        'state':  'controller[]/state'} )

    list_controller_types = ServiceMapper( 'pr2_mechanism_msgs', 'ListControllerTypes',  'controller_manager_msgs', 'ListControllerTypes',
                                           'pr2_controller_manager/list_controller_types', 'controller_manager/list_controller_types',
                                           { },
                                           { 'types':   'types' } )

    reload_controller_libraries = ServiceMapper( 'pr2_mechanism_msgs', 'ReloadControllerLibraries',  'controller_manager_msgs', 'ReloadControllerLibraries',
                                                 'pr2_controller_manager/reload_controller_libraries', 'controller_manager/reload_controller_libraries',
                                                 { 'force_kill': 'force_kill' },
                                                 { 'ok':   'ok' } )

    switch_controller = ServiceMapper( 'pr2_mechanism_msgs', 'SwitchController',  'controller_manager_msgs', 'SwitchController',
                                       'pr2_controller_manager/switch_controller', 'controller_manager/switch_controller',
                                       { 'start_controllers': 'start_controllers',
                                         'stop_controllers':  'stop_controllers',
                                         'strictness':        'strictness'},
                                       { 'ok':   'ok' } )

    unload_controller = ServiceMapper( 'pr2_mechanism_msgs', 'UnloadController',  'controller_manager_msgs', 'UnloadController',
                                       'pr2_controller_manager/unload_controller', 'controller_manager/unload_controller',
                                       { 'name': 'name' },
                                       { 'ok':   'ok'   } )

    rospy.spin()
