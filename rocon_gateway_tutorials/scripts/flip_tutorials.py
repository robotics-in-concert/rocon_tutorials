#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tutorials/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_gateway
from gateway_msgs.msg import *
from gateway_msgs.srv import *
import argparse
import sys

##############################################################################
# Utilities
##############################################################################

class Context(object):
    def __init__(self, gateway, cancel_flag, regex):
        self.gateway = gateway
        if cancel_flag:
            self.action_text = "cancelling"
        else:
            self.action_text = "flipping"
        self.flip_service = rospy.ServiceProxy('/gateway/flip',Remote)
        self.req = RemoteRequest()
        self.req.cancel = cancel_flag
        self.req.remotes = []
        self.names, self.nodes = rocon_gateway.samples.create_tutorial_dictionaries(use_regex_patterns=regex)

    def flip(self, type):
        rule = gateway_msgs.msg.Rule()
        rule.name = self.names[type]
        rule.type = type
        rule.node = self.nodes[type]
        self.req.remotes.append(RemoteRule(self.gateway,rule))
        rospy.loginfo("Flip : %s [%s,%s,%s,%s]."%(self.action_text,
                                                  self.gateway,
                                                  rule.type,
                                                  rule.name,
                                                  rule.node or 'None'))
        resp = self.flip_service(self.req)
        if resp.result != 0:
            rospy.logerr("Flip : %s"%resp.error_message)
        self.req.remotes = []

##############################################################################
# Main
##############################################################################

"""
  Tests flips, either for all tutorials (default) or one by one (via args).

  See the root readme for usage instructions.
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Flip roscpp tutorial connections (/chatter, /add_two_ints to a remote gateway')
    parser.add_argument('--pubonly', action='store_true', help='flip /chatter publisher only')
    parser.add_argument('--subonly', action='store_true', help='flip /chatter subscriber only')
    parser.add_argument('--serviceonly', action='store_true', help='flip add_two_ints service only')
    parser.add_argument('--actionclientonly', action='store_true', help='flip /fibonacci action client only')
    parser.add_argument('--actionserveronly', action='store_true', help='flip /averaging action server only')
    parser.add_argument('--regex', action='store_true', help='test with a regex pattern')
    parser.add_argument('--cancel', action='store_true', help='cancel the flip')
    argv = rospy.myargv(sys.argv)
    args = parser.parse_args(argv[1:])
    flip_all_connection_types = (not args.pubonly) and (not args.subonly) and (not args.serviceonly) and (not args.actionclientonly) and (not args.actionserveronly)
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "flipping"

    rospy.init_node('flip_tutorials')

    rocon_gateway.samples.wait_for_gateway()
    remote_gateway = rocon_gateway.samples.find_first_remote_gateway()
    context = Context(remote_gateway, args.cancel, args.regex)

    if args.pubonly or flip_all_connection_types:
        context.flip(ConnectionType.PUBLISHER)

    if args.subonly or flip_all_connection_types:
        context.flip(ConnectionType.SUBSCRIBER)

    if args.serviceonly or flip_all_connection_types:
        context.flip(ConnectionType.SERVICE)

    if args.actionclientonly or flip_all_connection_types:
        context.flip(ConnectionType.ACTION_CLIENT)

    if args.actionserveronly or flip_all_connection_types:
        context.flip(ConnectionType.ACTION_SERVER)
