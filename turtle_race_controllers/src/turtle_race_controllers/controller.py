#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/master/turtle_race_controllers/LICENSE
#
##############################################################################
# Imports
##############################################################################

import copy
#import math
import threading
import roslib
roslib.load_manifest('turtle_race_controllers')
import rospy
import tf
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

##############################################################################
# Exceptions
##############################################################################


class AlreadyRunningException(Exception):
    """
     If the worker thread is already running
    """
    pass

##############################################################################
# Classes
##############################################################################


class Transform(object):
    def __init__(self):
        self.translation = None  # list of doubles (size 3)
        self.rotation = None     # quaternion


class MotionLine(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.shutdown = False
        self._transform_broadcaster = tf.TransformBroadcaster()
        self._carrot_transform_name = "%s_carrot" % rospy.get_name()
        self._start_transform_name = "%s_start" % rospy.get_name()
        self._start_transform = Transform()
        self._carrot_transform = Transform()
        self._transform_listener = tf.TransformListener()

    def run(self):
        '''
          Catches the current base link transform and saves that as transform.
        '''
        while not self.shutdown and not rospy.is_shutdown():
            try:
                # from /odom -> /base_link
                (translation, rotation) = self._transform_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(0.25)
        self._start_transform.translation = list(translation)
        self._start_transform.rotation = list(rotation)
        self._carrot_transform = copy.deepcopy(self._start_transform)
        rate = rospy.Rate(50)
        while not self.shutdown and not rospy.is_shutdown():
            self._carrot_transform.translation[0] += 0.01
            rospy.loginfo("Moving: %s" % self._carrot_transform.translation[0])
            self._transform_broadcaster.sendTransform(self._start_transform.translation,
                         self._start_transform.rotation,
                         rospy.Time.now(),
                         self._start_transform_name,
                         "/odom")
            self._transform_broadcaster.sendTransform(self._carrot_transform.translation,
                         self._carrot_transform.rotation,
                         rospy.Time.now(),
                         self._carrot_transform_name,
                         self._start_transform_name,
                         )
            rate.sleep()


class RaceController(object):

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
        self._carrot_transform = Transform()
        self._worker_thread = None

        self._speed = 0.7
        self._distance = 1.0
        rospy.loginfo("Race Controller : initialised")
#        self._current_pose = Pose()
#        self._starting_pose = Pose()

    def shutdown(self):
        self._stop()
        self._cmd_vel_publisher.unregister()
        self._odom_subscriber.unregister()
        self._start_subscriber.unregister()
        self._stop_subscriber.unregister()

#    def execute(self):
#        '''
#          Drop this into threading.Thread or QThread for execution
#        '''
#        if self._running:
#            rospy.logerr("Kobuki TestSuite: already executing a motion, ignoring the request")
#            return
#        self._stop = False
#        self._running = True
#        rate = rospy.Rate(10)
#        self._current_speed = 0.0
#        current_distance_sq = 0.0
#        distance_sq = self.distance*self.distance
#        self._starting_pose = self._current_pose
#        while not self._stop and not rospy.is_shutdown():
#            if current_distance_sq > distance_sq:
#                break
#            else:
#                current_distance_sq = (self._current_pose.position.x - self._starting_pose.position.x)*(self._current_pose.position.x - self._starting_pose.position.x) + \
#                                   (self._current_pose.position.y - self._starting_pose.position.y)*(self._current_pose.position.y - self._starting_pose.position.y)
#                #current_distance_sq += 0.01 # uncomment this and comment above for debugging
#                print("Distance %s"%math.sqrt(current_distance_sq))
#                if self.speed > 0:
#                    if self._current_speed < self.speed:
#                        self._current_speed += 0.01
#                else:
#                    if self._current_speed > self.speed:
#                        self._current_speed -= 0.01
#                cmd = Twist()
#                cmd.linear.x = self._current_speed
#                self.cmd_vel_publisher.publish(cmd)
#            rate.sleep()
#        if not rospy.is_shutdown():
#            cmd = Twist()
#            cmd.linear.x = 0.0
#            self.cmd_vel_publisher.publish(cmd)
#        self._running = False

    def _stop(self):
        if self._worker_thread:
            self._worker_thread.shutdown = True
            self._worker_thread.join()
            self._worker_thread = None

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
        rospy.loginfo("Race Controller : received start command.")
        if self._worker_thread:
            raise AlreadyRunningException("The race has already started.")
        # Catch the bae link - odom transform here to initialise the carrot.
        self._worker_thread = MotionLine()
        self._worker_thread.start()

    def _stop_callback(self, unused_data):
        self._stop()

    def _odometry_callback(self, data):
        pass
