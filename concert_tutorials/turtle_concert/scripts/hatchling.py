#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to connect to the turtlesim engine and 'hatch'. 
#
# - watch the app manager status and when it has a remote controller,
# - flip a spawn/kill pair across
# - call the spawn api
#  - the turtle herder will flip back some handles then.

##############################################################################
# Imports
##############################################################################

import copy
import rospy
import rocon_python_comms
import rocon_gateway
import rocon_tutorial_msgs.msg as rocon_tutorial_msgs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import std_msgs.msg as std_msgs
import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import threading

##############################################################################
# Classes
##############################################################################

class Hatchling:
    '''
      Sits in the concert client and makes the connections to the turtlesim engine.
    '''
    __slots__ = [
        'name',
        'spawn_turtle',
        'kill_turtle',
        'gateway_flip_service',
        'remote_controller',  # cached variable holding the current name of the remote controller
        'new_remote_controller',  # cached variable holding the new remote controller pending a spawn activity
        'lock',
    ]

    def __init__(self):
        # could delay this till we have a remote controller and 'sniff' instead of hardcoding
        self.spawn_turtle = rocon_python_comms.ServicePairClient('~spawn', rocon_tutorial_msgs.SpawnTurtlePair)
        self.kill_turtle = rocon_python_comms.ServicePairClient('~kill', rocon_tutorial_msgs.KillTurtlePair)
        # gateway
        gateway_namespace = rocon_gateway.resolve_local_gateway()
        self.gateway_flip_service = rospy.ServiceProxy(gateway_namespace + '/flip', gateway_srvs.Remote)
        self.name = rocon_gateway.resolve_gateway_info(gateway_namespace).name
        # app manager
        rospy.Subscriber('~remote_controller', std_msgs.String, self._ros_subscriber_remote_controller)
        self.remote_controller = ''
        self.new_remote_controller = None
        self.lock = threading.Lock()

    def _flip_rules(self):
        rules = []
        rule = gateway_msgs.Rule()
        rule.node = rospy.get_name()
        rule.type = gateway_msgs.ConnectionType.PUBLISHER
        rule.name = rospy.resolve_name("~spawn/request")
        rules.append(copy.deepcopy(rule))
        rule.name = rospy.resolve_name("~kill/request")
        rules.append(copy.deepcopy(rule))
        rule.type = gateway_msgs.ConnectionType.SUBSCRIBER
        rule.name = rospy.resolve_name("~spawn/response")
        rules.append(copy.deepcopy(rule))
        rule.name = rospy.resolve_name("~kill/response")
        return rules

    def _ros_subscriber_remote_controller(self, msg):
        '''
          Callback for the app manager's latched remote controller publisher that informs us of it's
          changes in state.
        '''
        self.lock.acquire()
        self.new_remote_controller = msg.data
        request = gateway_srvs.RemoteRequest()
        request.cancel = False if self.new_remote_controller else True
        remote_rule = gateway_msgs.RemoteRule()
        remote_rule.gateway = self.new_remote_controller
        for rule in self._flip_rules():
            remote_rule.rule = rule
            request.remotes.append(copy.deepcopy(remote_rule))
        self.lock.release()
        try:
            response = self.gateway_flip_service(request)
        except rospy.ServiceException:  # communication failed
            rospy.logerr("Hatchling : failed to send flip rules")
            return
        except rospy.ROSInterruptException:
            rospy.loginfo("Hatchling : shutdown while contacting the gateway flip service")
            return

    def spin(self):
        '''
          Loop around checking if there's work to be done registering our hatchling on the turtlesim engine.
        '''
        while not rospy.is_shutdown():
            self.lock.acquire()
            if self.new_remote_controller is not None:
                if self.new_remote_controller == '':
                    response = self.kill_turtle(rocon_tutorial_msgs.KillTurtleRequest(self.name), timeout=rospy.Duration(3.0))
                else:
                    response = self.spawn_turtle(rocon_tutorial_msgs.SpawnTurtleRequest(self.name), timeout=rospy.Duration(3.0))
                self.new_remote_controller = None
            self.lock.release()
            rospy.rostime.wallsleep(0.5)
            

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('hatchling')
    hatchling = Hatchling()
    hatchling.spin()
