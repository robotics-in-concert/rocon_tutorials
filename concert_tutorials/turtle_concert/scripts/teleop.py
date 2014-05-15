#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple teleop for a turtlesim turtle. It relays cmd_vel's to the
# turtlesim cmd_vel underneath and also publishes a fake compressed image view
# (real robots would send an image stream).

##############################################################################
# Imports
##############################################################################

import rospy
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs
import rocon_python_utils

##############################################################################
# Classes
##############################################################################


class TurtleTeleop:
    '''
      Shepherds the turtles! This is the implementation node for the
      turtle_concert/teleop rapp.

      @todo get alised names from the concert client list if the topic is available

      @todo watchdog for killing turtles that are no longer connected.
    '''
    __slots__ = [
        'simulation_namespace',
        'cmd_vel_subscriber',
        'cmd_vel_publisher',
        'image_publisher',
    ]

    def __init__(self):
        try:
            self.simulation_namespace = rospy.get_param("~simulation_namespace")
        except KeyError:
            rospy.logerr("Turtle Teleop : failed to get the turtle name from the parameter server.")
        # Set up a simple relay here to the underlying cmd vel handle to the turtlesim engine.
        self.cmd_vel_subscriber = rospy.Subscriber("cmd_vel", geometry_msgs.Twist, self.ros_cmd_vel_callback)
        self.cmd_vel_publisher = rospy.Publisher(self.simulation_namespace + '/cmd_vel', geometry_msgs.Twist, queue_size=5)
        self.image_publisher = rospy.Publisher('compressed_image', sensor_msgs.CompressedImage, latch=True, queue_size=1)
        self.publish_teleop_image()

    def ros_cmd_vel_callback(self, msg):
        '''
          Just relay this to the turtlesim engine's handle.
        '''
        self.cmd_vel_publisher.publish(msg)

    def publish_teleop_image(self):
        '''
          Currently we only do this once as it is a static image.
        '''
        msg = sensor_msgs.CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'png'
        image_filename = rocon_python_utils.ros.find_resource_from_string('rocon_bubble_icons/turtle_teleop.png')
        msg.data = open(image_filename, "rb").read()
        self.image_publisher.publish(msg)

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':

    rospy.init_node('turtle_teleop')

    turtle_teleop = TurtleTeleop()
    rospy.spin()
