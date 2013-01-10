#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/master/turtle_race_controllers/LICENSE
#
##############################################################################
# Imports
##############################################################################

import math
import threading
import random
import roslib
roslib.load_manifest('turtle_race_controllers')
import rospy
import tf
from geometry_msgs.msg import Twist, Pose
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
        self.translation = [0.0, 0.0, 0.0]    # list of doubles (size 3)
        self.rotation = [0.0, 0.0, 0.0, 1.0]  # unit quaternion


class DoinTheWal(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._cmd_vel_publisher = rospy.Publisher("~cmd_vel", Twist)
        self.shutdown = False

    def run(self):
        '''
          Dance that is triggered once its won.
          Just wiggles its butt left and right
        '''
        rospy.loginfo("Turtle Race : doin the wal!")
        start_time = rospy.Time.now()
        interval = rospy.Duration(0.5)  # seconds, do something every 0.1s
        total_duration = rospy.Duration(3.0)
        progress_landmark = rospy.Duration(0.0) + interval
        twist = Twist()
        twist.angular.z = 0.5
        while not self.shutdown and not rospy.is_shutdown():
            duration = rospy.Time.now() - start_time
            if duration > total_duration:
                break
            if duration > progress_landmark:
                progress_landmark += interval
                twist.angular.z = -1 * twist.angular.z
            self._cmd_vel_publisher.publish(twist)
        twist.linear.z = 0.0
        self._cmd_vel_publisher.publish(twist)
        self._do_the_wal_flag = False
        rospy.loginfo("Turtle Race : finished the wal")


class DragRace(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        # Transforms
        self._transform_broadcaster = tf.TransformBroadcaster()
        self._carrot_transform_name = "/carrot"  # %s/carrot" % rospy.get_name()
        self._start_transform_name = "/race_start"  # "%s/start" % rospy.get_name()
        self._start_transform = Transform()
        self._carrot_transform = Transform()
        self._transform_listener = tf.TransformListener()
        # Ros api
        self._cmd_vel_publisher = rospy.Publisher("~cmd_vel", Twist)
        self._finished_publisher = rospy.Publisher("race_finished", Empty)
        self.shutdown = False

    def run(self):
        '''
          Racing control
        '''
        while not self.shutdown and not rospy.is_shutdown():
            translation = None
            try:
                # from /odom -> /base_link
                (translation, rotation) = self._transform_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))  # from, to
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print type(e)
                print e
                rospy.sleep(0.25)
        self._start_transform.translation = list(translation)
        self._start_transform.rotation = list(rotation)
        self._carrot_transform = Transform()  # initialise as the zero transform, not copy.deepcopy(self._start_transform)
        frequency = 50
        rate = rospy.Rate(frequency)
        goal_distance = 1.0  # x distance it must travel
        goal_overshoot = 0.05
        goal_error_condition = 0.05  # must get this close
        landmark = 0.1  # establish landmarks every 0.1m
        translation_step = 0.3 / frequency
        while not self.shutdown and not rospy.is_shutdown():
            if self._carrot_transform.translation[0] < goal_distance + goal_overshoot:  # must dangle the carrot
                if self._carrot_transform.translation[0] > landmark:
                    landmark += 0.1
                    translation_step = (0.3 + random.uniform(0.0, 0.2)) / frequency
                # Update the carrot
                self._carrot_transform.translation[0] += translation_step
                rospy.loginfo("Turtle Race : moving the carrot: %s" % self._carrot_transform.translation[0])
            # Publish nonetheless, tf likes getting spammed
            self._transform_broadcaster.sendTransform(
                         self._start_transform.translation,
                         self._start_transform.rotation,
                         rospy.Time.now(),
                         self._start_transform_name,
                         "/odom")
            self._transform_broadcaster.sendTransform(
                         self._carrot_transform.translation,
                         self._carrot_transform.rotation,
                         rospy.Time.now(),
                         self._carrot_transform_name,
                         self._start_transform_name,
                         )
            # Get the control
            try:
                (translation, rotation) = self._transform_listener.lookupTransform('/base_link', self._carrot_transform_name, rospy.Time(0))
                twist = Twist()
                # Use a better than proportional control here.
                twist.angular.z = 4 * math.atan2(translation[1], translation[0])
                twist.linear.x = 0.5 * math.sqrt(translation[0] ** 2 + translation[1] ** 2)
                self._cmd_vel_publisher.publish(twist)
                translation_error = translation[0]
                rospy.loginfo("Turtle Race : distance to go [%s]" % translation_error)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("Turtle Race : could not generate command velocities from transform %s", e)
            # Abort if goal is reached
            if math.fabs(translation[0]) < goal_error_condition and self._carrot_transform.translation[0] > goal_distance:
                rospy.loginfo("Turtle Race : finished!")
                twist = Twist()
                self._cmd_vel_publisher.publish(twist)     # zero the cmd_vel
                self._finished_publisher.publish(Empty())  # ping the concert
                break
            rate.sleep()


class RaceController(object):
    '''
      Initialise everything

      @param topic names
      @type strings
    '''
    def __init__(self):
        self._start_subscriber = rospy.Subscriber("race_start", Empty, self._start_callback)
        self._won_subscriber = rospy.Subscriber("race_winner", Empty, self._race_won_callback)

        self._doin_the_wal = None
        self._race = None
        rospy.loginfo("Turtle Race : ready to rumble! [%s]" % rospy.get_name())

    def shutdown(self):
        self._stop()
        self._start_subscriber.unregister()
        self._won_subscriber.unregister()

    def _stop(self):
        if self._doin_the_wal:
            self._doin_the_wal.shutdown = True
            self._doin_the_wal.join()
            self._doin_the_wal.shutdown = False
        if self._race:
            self._race.shutdown = True
            self._race.join()
            self._race.shutdown = False

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def _start_callback(self, unused_data):
        rospy.loginfo("Race Controller : received start command.")
        if self._doin_the_wal or self._race:
            raise AlreadyRunningException("The race has already started.")
        self._race = DragRace()
        self._race.start()

    def _race_won_callback(self, unused_data):
        if self._doin_the_wal or self._race:
            raise AlreadyRunningException("The race has already started.")
        rospy.loginfo("Turtle Race : you won - let's do the wal!")
        self._doin_the_wal = DoinTheWal()
        self._doin_the_wal.start()
