#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/master/turtle_race/LICENSE
#
##############################################################################
# Imports
##############################################################################

import math
import roslib
roslib.load_manifest('turtle_race_controllers')
import rospy
import tf
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

##############################################################################
# Classes
##############################################################################


class RaceController(object):

    class Transform(object):
        def __init__(self):
            self.translation = None  # list of doubles (size 3)
            self.rotation = None     # rotation 
    '''
      Initialise everything

      @param topic names
      @type strings
    '''
    def __init__(self):
        self._odom_subscriber = rospy.Subscriber("~odom", Odometry, self._odometry_callback)
        self._cmd_vel_publisher = rospy.Publisher("~cmd_vel", Twist)
        # should use an action server instead
        self._start_subscriber = rospy.Subscriber("~start", Empty, self._start_callback)
        self._stop_subscriber = rospy.Subscriber("~stop", Empty, self._stop_callback)
        self._transform_listener = tf.TransformListener()
        self._carrot_transform = RaceController.Transform()

        self._speed = 0.7
        self._distance = 1.0
        self._current_pose = Pose()
        self._starting_pose = Pose()
        self._stop = False
        self._running = False

    def init(self, speed, distance):
        self.speed = speed
        self.distance = distance

    def shutdown(self):
        self.stop()
        while self._running:
            rospy.sleep(0.05)
        self.cmd_vel_publisher.unregister()
        self.odom_subscriber.unregister()

    def stop(self):
        self._stop = True

    def execute(self):
        '''
          Drop this into threading.Thread or QThread for execution
        '''
        if self._running:
            rospy.logerr("Kobuki TestSuite: already executing a motion, ignoring the request")
            return
        self._stop = False
        self._running = True
        rate = rospy.Rate(10)
        self._current_speed = 0.0
        current_distance_sq = 0.0
        distance_sq = self.distance*self.distance
        self._starting_pose = self._current_pose
        while not self._stop and not rospy.is_shutdown():
            if current_distance_sq > distance_sq:
                break
            else:
                current_distance_sq = (self._current_pose.position.x - self._starting_pose.position.x)*(self._current_pose.position.x - self._starting_pose.position.x) + \
                                   (self._current_pose.position.y - self._starting_pose.position.y)*(self._current_pose.position.y - self._starting_pose.position.y)
                #current_distance_sq += 0.01 # uncomment this and comment above for debugging
                print("Distance %s"%math.sqrt(current_distance_sq))
                if self.speed > 0:
                    if self._current_speed < self.speed:
                        self._current_speed += 0.01
                else:
                    if self._current_speed > self.speed:
                        self._current_speed -= 0.01
                cmd = Twist()
                cmd.linear.x = self._current_speed
                self.cmd_vel_publisher.publish(cmd)
            rate.sleep()
        if not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = 0.0
            self.cmd_vel_publisher.publish(cmd)
        self._running = False

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

#    def cliff_sensor_callback(self, data):
#        rospy.loginfo("Kobuki Testsuite: cliff event on sensor [%s]"%str(data.sensor))
#        if data.state == CliffEvent.CLIFF:
#            if not rospy.is_shutdown():
#                cmd = Twist()
#                cmd.linear.x = 0.0
#                self.cmd_vel_publisher.publish(cmd)
#            self.stop()

    def _start_callback(self, unused_data):
        # Catch the bae link - odom transform here to initialise the carrot.
        try:
            (self._carrot_transform.translation, self._carrot_transform.rotation) = self._transform_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        self._is_running = True

    def _stop_callback(self, unused_data):
        self._current_pose = data.pose.pose
        self._is_running = False
