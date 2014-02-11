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

##############################################################################
# Methods
##############################################################################

def resolve_local_gateway():
    master = rocon_gateway.LocalMaster()
    gateway_namespace = master.find_gateway_namespace()
    if not gateway_namespace:
        raise rocon_gateway.GatewayError("Could not find a local gateway - did you start it?")
    #console.debug("Found a local gateway at %s"%gateway_namespace)
    return gateway_namespace

def resolve_gateway_info(gateway_namespace):
    '''
      @raise rocon_gateway.GatewayError: if no remote gateways or no matching gateways available. 
    '''
    gateway_info = rocon_python_comms.SubscriberProxy(gateway_namespace+'/gateway_info', gateway_msgs.msg.GatewayInfo)()
    return gateway_info

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
        'gateway_flip_service'
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
        rospy.logwarn("initialised")

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
        request = gateway_srvs.RemoteRequest()
        request.cancel = False if msg.data else True
        remote_rule = gateway_msgs.RemoteRule()
        remote_rule.gateway = msg.data
        for rule in self._flip_rules():
            remote_rule.rule = rule
            request.remotes.append(copy.deepcopy(remote_rule))
        rospy.logwarn("Publishing to flip service")
        try:
            response = self.gateway_flip_service(request)
        except rospy.ServiceException:  # communication failed
            rospy.logerr("Hatchling : failed to send flip rules")
            return
        except rospy.ROSInterruptException:
            rospy.loginfo("Hatchling : shutdown while contacting the gateway flip service")
            return
        #response = spawn_turtle(rocon_tutorial_msgs.SpawnTurtleRequest('kobuki'), timeout=rospy.Duration(3.0))
            

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('hatchling')
    hatchling = Hatchling()
    while not rospy.is_shutdown():
        rospy.rostime.wallsleep(0.5)
#     turtle_herder = TurtleHerder()
#     spawn_turtle = rocon_python_comms.ServicePairClient('spawn', rocon_tutorial_msgs.SpawnTurtlePair)
#     rospy.rostime.wallsleep(0.5)
#     request = rocon_tutorial_msgs.SpawnTurtleRequest('kobuki')
#     print("Request: %s" % request)
#     response = spawn_turtle(rocon_tutorial_msgs.SpawnTurtleRequest('kobuki'), timeout=rospy.Duration(3.0))
#     response = spawn_turtle(rocon_tutorial_msgs.SpawnTurtleRequest('guimul'), timeout=rospy.Duration(3.0))
#     print("Response: %s" % response)
    rospy.spin()
    #turtle_herder.shutdown()
