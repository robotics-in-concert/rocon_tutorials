#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/turtle_concert/master/LICENSE
#
##############################################################################
# Imports
##############################################################################

# Simple script to clear the turtlesim display on startup.

import rospy
import turtlesim.srv as turtlesim_srvs

rospy.init_node('clear_turtles')
rospy.wait_for_service('kill')
kill_turtle = rospy.ServiceProxy('kill', turtlesim_srvs.Kill)
response = kill_turtle("turtle1")

# Only because we can't get services publicly exposed yet.
# The master sync can do it, but am not yet doing it
rospy.wait_for_service('spawn')
spawn_turtle = rospy.ServiceProxy('spawn', turtlesim_srvs.Spawn)
response = spawn_turtle(5.4,6.4,0.0,"turtle_one")
response = spawn_turtle(5.4,4.4,0.3,"turtle_two")
