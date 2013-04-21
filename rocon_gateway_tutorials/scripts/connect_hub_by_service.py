#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_gateway_tutorials/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_gateway
import sys

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('connect_hub_by_service')
    try:
        rocon_gateway.samples.connect_hub_by_service()
    except rocon_gateway.GatewaySampleRuntimeError as e:
        rospy.logerr("Hub Connector: %s" % str(e))
        sys.exit(1)
