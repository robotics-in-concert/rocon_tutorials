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
#
# It supplements this relay role with a bit of herd management - sets up
# random start locations and feeds back aliased names when running with
# a concert.

##############################################################################
# Imports
##############################################################################

import math
import random

import rospy
import rocon_python_comms

import turtlesim.srv as turtlesim_srvs
import rocon_tutorial_msgs.msg as rocon_tutorial_msgs

##############################################################################
# Classes
##############################################################################

class TurtleHerder:
    '''
      Shepherds the turtles!
      
      @todo get alised names from the concert client list if the topic is available

      @todo watchdog for killing turtles that are no longer connected.
    '''
    __slots__ = [
        'turtles',  # Dictionary of string : concert_msgs.RemoconApp[]
    ]
    
    def __init__(self):
        self.turtles = []
        rospy.wait_for_service('~kill')  # could use timeouts here
        rospy.wait_for_service('~spawn')
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
            self.turtles.remove(msg.name)
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
        internal_service_request = turtlesim_srvs.SpawnRequest(
                                            random.uniform(4.0, 6.0),
                                            random.uniform(4.0, 6.0),
                                            random.uniform(0.0, 2.0 * math.pi),
                                            msg.name)
        try:
            internal_service_response = self._spawn_turtle_service_client(internal_service_request)
            self.turtles.append(msg.name)
        except rospy.ServiceException:  # communication failed
            rospy.logerr("Spawn Turtles : failed to contact the internal spawn turtle service")
        except rospy.ROSInterruptException:
            rospy.loginfo("Spawn Turtles : shutdown while contacting the internal spawn turtle service")
            return
        self._spawn_turtle_service_pair_server.reply(request_id, response)

    def shutdown(self):
        for turtle in self.turtles:
            try:
                internal_service_response = self._kill_turtle_service_client(msg.name)
            except rospy.ServiceException:  # communication failed
                break  # quietly fail
            except rospy.ROSInterruptException:
                break  # quietly fail

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('spawn_turtles')
    turtle_herder = TurtleHerder()
    rospy.spin()
    turtle_herder.shutdown()
