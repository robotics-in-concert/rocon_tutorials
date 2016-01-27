#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tutorials/LICENSE
#

import rospy
import rocon_gateway
from gateway_msgs.msg import *
from gateway_msgs.srv import *
import argparse
import sys

"""
  Advertises everything. See the root readme for usage instructions.
"""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Advertise all connections (unadvertise if using --cancel')
    parser.add_argument('--cancel', action='store_true', help='cancel the advertisements')
    args = parser.parse_args()
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "advertising"

    rospy.init_node('advertise_all')

    advertise_all = rospy.ServiceProxy('/gateway/advertise_all',AdvertiseAll)
    req = AdvertiseAllRequest()
    req.cancel = args.cancel
    req.blacklist = []

    rospy.loginfo("Advertise All : %s all."%action_text)
    resp = advertise_all(req)
    if resp.result != 0:
        rospy.logerr("Advertise All : error occured (todo: no error message yet)")
        #rospy.logerr("Advertise : %s"%resp.error_message)

