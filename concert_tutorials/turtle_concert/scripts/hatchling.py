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
        'remote_controller_updates',  # cached list holding new remote controller updates pending a spawn activity
        'event_remote_controller_changed',
    ]

    def __init__(self):
        self.event_remote_controller_changed = threading.Event()
        # could delay this till we have a remote controller and 'sniff' instead of hardcoding
        self.spawn_turtle = rocon_python_comms.ServicePairClient('spawn', rocon_tutorial_msgs.SpawnTurtlePair)
        self.kill_turtle = rocon_python_comms.ServicePairClient('kill', rocon_tutorial_msgs.KillTurtlePair)
        # gateway
        gateway_namespace = rocon_gateway.resolve_local_gateway()
        self.gateway_flip_service = rospy.ServiceProxy(gateway_namespace + '/flip', gateway_srvs.Remote)
        self.name = rocon_gateway.resolve_gateway_info(gateway_namespace).name
        # app manager
        rospy.Subscriber('remote_controller', std_msgs.String, self._ros_subscriber_remote_controller)
        self.remote_controller = rocon_app_manager_msgs.Constants.NO_REMOTE_CONTROLLER
        self.remote_controller_updates = []

    def _flip_rules(self):
        rules = []
        rule = gateway_msgs.Rule()
        rule.node = rospy.get_name()
        rule.type = gateway_msgs.ConnectionType.PUBLISHER
        rule.name = rospy.resolve_name("spawn/request")
        rules.append(copy.deepcopy(rule))
        rule.name = rospy.resolve_name("kill/request")
        rules.append(copy.deepcopy(rule))
        rule.type = gateway_msgs.ConnectionType.SUBSCRIBER
        rule.name = rospy.resolve_name("spawn/response")
        rules.append(copy.deepcopy(rule))
        rule.name = rospy.resolve_name("kill/response")
        rules.append(copy.deepcopy(rule))
        return rules

    def _send_flip_rules(self, remote_controller):
        request = gateway_srvs.RemoteRequest()
        request.cancel = True if remote_controller == rocon_app_manager_msgs.Constants.NO_REMOTE_CONTROLLER else False
        remote_rule = gateway_msgs.RemoteRule()
        if request.cancel:
            remote_rule.gateway = self.remote_controller
        else:
            remote_rule.gateway = remote_controller
        for rule in self._flip_rules():
            remote_rule.rule = rule
            request.remotes.append(copy.deepcopy(remote_rule))
        try:
            response = self.gateway_flip_service(request)
        except rospy.ServiceException:  # communication failed
            rospy.logerr("Hatchling : failed to send flip rules")
            return
        except rospy.ROSInterruptException:
            rospy.loginfo("Hatchling : shutdown while contacting the gateway flip service")
            return

    def _ros_subscriber_remote_controller(self, msg):
        '''
          Callback for the app manager's latched remote controller publisher that informs us of it's
          changes in state.
        '''
        # only using thread-safe list operations: http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm
        self.remote_controller_updates.append(msg.data)
        self.event_remote_controller_changed.set()

    def spin(self):
        '''
          Loop around checking if there's work to be done registering our hatchling on the turtlesim engine.
        '''
        interacting_with_remote_controller = None
        while not rospy.is_shutdown():
            event_is_set = self.event_remote_controller_changed.wait(0.5)
            if event_is_set:  # clear it so that it blocks again at the next run.
                self.event_remote_controller_changed.clear()
            if interacting_with_remote_controller is None: # check if we have a remote controller state change and send flips
                # Just send off one set of flips at a time, have to make sure we process kill/spawn
                if len(self.remote_controller_updates) > 0:
                    try:
                        interacting_with_remote_controller = self.remote_controller_updates.pop(0)
                        if interacting_with_remote_controller == self.remote_controller:
                            interacting_with_remote_controller = None  # do nothing
                        else:
                            self._send_flip_rules(interacting_with_remote_controller)
                    except IndexError:
                        rospy.logerr("Hatchling: index error")
            else:  # see if we can register with turtlesim or not yet.
                if interacting_with_remote_controller == rocon_app_manager_msgs.Constants.NO_REMOTE_CONTROLLER:
                    response = self.kill_turtle(rocon_tutorial_msgs.KillTurtleRequest(self.name), timeout=rospy.Duration(4.0))
                else:
                    response = self.spawn_turtle(rocon_tutorial_msgs.SpawnTurtleRequest(self.name), timeout=rospy.Duration(4.0))
                if response:  # didn't time out, probably waiting for the flips to arrive.
                    self.remote_controller = interacting_with_remote_controller
                    interacting_with_remote_controller = None
            

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('hatchling')
    hatchling = Hatchling()
    hatchling.spin()
