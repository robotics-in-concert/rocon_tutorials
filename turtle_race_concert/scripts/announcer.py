#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/master/turtle_race_concert/LICENSE
#
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('turtle_race_concert')
import rospy
from std_msgs.msg import Empty


##############################################################################
# Callbacks
##############################################################################

class Announcer(object):

    def __init__(self):
        self._race_finished = False
        self._winner_publishers = {}
        self._winner_publishers['race_winner_one']   = rospy.Publisher("race_winner_one", Empty)
        self._winner_publishers['race_winner_two']   = rospy.Publisher("race_winner_two", Empty)
        self._winner_publishers['race_winner_three'] = rospy.Publisher("race_winner_three", Empty)
        self._start_publisher = rospy.Publisher("race_start", Empty)
        self._finished_subscribers = {}
        self._finished_subscribers['race_finished_one']   = rospy.Subscriber("race_finished_one",   Empty, self._race_finished_one_callback)
        self._finished_subscribers['race_finished_two']   = rospy.Subscriber("race_finished_two",   Empty, self._race_finished_two_callback)
        self._finished_subscribers['race_finished_three'] = rospy.Subscriber("race_finished_three", Empty, self._race_finished_three_callback)

    def start(self):
        self._race_finished = False
        self._start_publisher.publish(Empty())

    ##################################
    # Callbacks
    ##################################
    # These should be merged and use something smart to distinguish - either 
    # msg data or resolved namespace or something

    def _race_finished_one_callback(self, data):
        if self._race_finished:
            rospy.loginfo("Turtle Race Announcer : turtle one has won!")
            self._winner_publishers['race_winner_one'].publish(Empty())
    
    def _race_finished_two_callback(self, data):
        if self._race_finished:
            rospy.loginfo("Turtle Race Announcer : turtle two has won!")
            self._winner_publishers['race_winner_two'].publish(Empty())
    
    def _race_finished_three_callback(self, data):
        if self._race_finished:
            rospy.loginfo("Turtle Race Announcer : turtle three has won!")
            self._winner_publishers['race_winner_three'].publish(Empty())

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('turtle_race_announcer')
    announcer = Announcer()
    for num in range(20):
        rospy.loginfo("Turtle Race Announcer : start in %s" % (20 - num))
        rospy.sleep(1)
    announcer.start()
    rospy.spin()
