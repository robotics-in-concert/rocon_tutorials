#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to manage spawning and killing of turtles across multimaster
# boundaries. Typically turtlesim clients would connect to the kill and
# spawn services directly to instantiate themselves, but since we can't
# flip service proxies, this is not possible. So this node is the inbetween
# go-to node and uses a rocon service pair instead.

##############################################################################
# Imports
##############################################################################

import rospy
import rocon_python_comms

import turtlesim.srv as turtlesim_srvs
import rocon_tutorial_msgs.msg as rocon_tutorial_msgs

##############################################################################
# Classes
##############################################################################

class TurtleManagement:
    
    def __init__(self):
        rospy.wait_for_service('kill')  # could use timeouts here
        rospy.wait_for_service('spawn')
        self._spawn_turtle_service_client = rospy.ServiceProxy('~spawn', turtlesim_srvs.Spawn, persistent=True)
        self._kill_turtle_service_client = rospy.ServiceProxy('~kill', turtlesim_srvs.Kill, persistent=True)
        # kill the default turtle that turtlesim starts with
        try:
            response = self._kill_turtle_service_client("turtle1")
        except rospy.ServiceException:
            rospy.logerr("Spawn Turtles : failed to contact the internal kill turtle service")
        except rospy.ROSInterruptException:
            rospy.loginfo("Spawn Turtles : shutdown while contacting the internal kill turtle service")
            return
        self._kill_turtle_service_pair_server = rocon_python_comms.ServicePairServer('kill', self._kill_turtle_service, rocon_tutorial_msgs.KillTurtlePair)
        self._spawn_turtle_service_pair_server = rocon_python_comms.ServicePairServer('spawn', self._spawn_turtle_service, rocon_tutorial_msgs.SpawnTurtlePair)

    def _kill_turtle_service(self, request_id, msg):
        '''
          @param request_id
          @type uuid_msgs/UniqueID
          @param msg
          @type ServiceRequest
        '''
        response = rocon_tutorial_msgs.KillTurtleResponse()
        internal_service_request = turtlesim_srvs.KillRequest(msg.name)
        try:
            internal_service_response = self._kill_turtle_service_client(internal_service_request)
        except rospy.ServiceException:  # communication failed
            rospy.logerr("Spawn Turtles : failed to contact the internal kill turtle service")
        except rospy.ROSInterruptException:
            rospy.loginfo("Spawn Turtles : shutdown while contacting the internal kill turtle service")
            return
        self._kill_turtle_service_pair_server.reply(request_id, response)
        
    def _spawn_turtle_service(self, request_id, msg):
        '''
          @param request_id
          @type uuid_msgs/UniqueID
          @param msg
          @type ServiceRequest
        '''
        response = rocon_tutorial_msgs.SpawnTurtleResponse()
        self._spawn_turtle_service_pair_server.reply(request_id, response)
        internal_service_request = turtlesim_srvs.SpawnRequest(msg.x, msg.y, msg.theta, msg.name)
        try:
            internal_service_response = self._spawn_turtle_service_client(internal_service_request)
        except rospy.ServiceException:  # communication failed
            rospy.logerr("Spawn Turtles : failed to contact the internal spawn turtle service")
        except rospy.ROSInterruptException:
            rospy.loginfo("Spawn Turtles : shutdown while contacting the internal spawn turtle service")
            return
        self._spawn_turtle_service_pair_server.reply(request_id, response)

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('spawn_turtles')
    turtle_management = TurtleManagement()

    # spawn some turtles for testing.
    #self.testies = rocon_python_comms.ServicePairClient('testies', rocon_service_pair_msgs.TestiesPair)
    #response = spawn_turtle(5.4,6.4,0.0,"turtle_one")
    #response = spawn_turtle(5.4,4.4,0.3,"turtle_two")

    rospy.spin()
