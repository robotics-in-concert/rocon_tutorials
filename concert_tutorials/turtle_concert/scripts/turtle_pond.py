#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to request a pre-configured number of strolling turtles.
# This could also be done with link graphs, but chatter concert does that.

##############################################################################
# Imports
##############################################################################

import rospy
import rocon_python_comms
import concert_service_utilities
import concert_scheduler_requests
import unique_id
import scheduler_msgs.msg as scheduler_msgs
import concert_msgs.msg as concert_msgs
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_uri

##############################################################################
# Classes
##############################################################################


class TurtlePond:
    '''
      Requets handling for getting its mittens on some turtle_stroll'ing
      turtles.
    '''
    __slots__ = [
        'service_name',
        'service_description',
        'service_priority',
        'service_id',
        'requester',
        'pending_requests',   # [uuid.UUID] list of request id's pending feedback from the scheduler
        'allocated_requests'  # [uuid.UUID] moved here from pending requests once granted
    ]

    def __init__(self):
        ####################
        # Discovery
        ####################
        (self.service_name, self.service_description, self.service_priority, self.service_id) = concert_service_utilities.get_service_info()

        ####################
        # Setup
        ####################
        self.requester = self.setup_requester(self.service_id)
        self.pending_requests = []
        self.allocated_requests = []

        turtles = rospy.get_param("turtles", default=[{'x_vel':1.0,'z_vel':0.4,'square_scale':1.0}])

        rospy.loginfo("TurtlePond : requesting %s turtles" % len(turtles))
        for turtle in turtles:
            x_vel = turtle['x_vel']
            z_vel = turtle['z_vel']
            scale = turtle['square_scale']
            self.request_turtle(x_vel, z_vel, scale)

    def setup_requester(self, uuid):
        try:
            scheduler_requests_topic_name = concert_service_utilities.find_scheduler_requests_topic()
            #rospy.loginfo("Service : found scheduler [%s][%s]" % (topic_name))
        except rocon_python_comms.NotFoundException as e:
            rospy.logerr("TurtlePond : %s" % (str(e)))
            return  # raise an exception here?
        frequency = concert_scheduler_requests.common.HEARTBEAT_HZ
        return concert_scheduler_requests.Requester(self.requester_feedback, uuid, 0, scheduler_requests_topic_name, frequency)

    def request_turtle(self, x_vel=0.1, z_vel=0.1, scale=1.0):
        '''
         Request a turtle.
        '''
        resource = scheduler_msgs.Resource()
        resource.id = unique_id.toMsg(unique_id.fromRandom())
        resource.rapp = 'turtle_concert/turtle_stroll'
        resource.uri = 'rocon:/'
        resource_request_id = self.requester.new_request([resource], priority=self.service_priority)
        resource.parameters = [rocon_std_msgs.KeyValue('turtle_x_vel', str(x_vel)), rocon_std_msgs.KeyValue('turtle_z_vel', str(z_vel)), rocon_std_msgs.KeyValue('square_scale', str(scale))]

        self.pending_requests.append(resource_request_id)
        self.requester.send_requests()



    def requester_feedback(self, request_set):
        '''
          Keep an eye on our pending requests and see if they get allocated here.
          Once they do, kick them out of the pending requests list so _ros_capture_teleop_callback
          can process and reply to the interaction.

          @param request_set : the modified requests
          @type dic { uuid.UUID : scheduler_msgs.ResourceRequest }
        '''
        cancelled_requests = 0
        for request_id, request in request_set.requests.iteritems():
            if request_id in self.pending_requests:
                if request.msg.status == scheduler_msgs.Request.GRANTED:
                    self.pending_requests.remove(request_id)
                    self.allocated_requests.append(request_id)
            elif request.msg.status == scheduler_msgs.Request.GRANTED:
                completely_unallocated = True
                for resource in request.msg.resources:
                    if rocon_uri.parse(resource.uri).name.string != concert_msgs.Strings.SCHEDULER_UNALLOCATED_RESOURCE:
                        completely_unallocated = False
                if completely_unallocated:
                    cancelled_requests += 1
                    request.cancel()
            # Need to add logic here for when the request gets released
        for unused_i in range(0, cancelled_requests):
            self.request_turtle()

    def cancel_all_requests(self):
        '''
          Exactly as it says! Used typically when shutting down or when
          it's lost more allocated resources than the minimum required (in which case it
          cancels everything and starts reissuing new requests).
        '''
        self.requester.cancel_all()
        self.requester.send_requests()

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':

    rospy.init_node('turtle_pond')

    turtle_pond = TurtlePond()
    rospy.spin()
    if not rospy.is_shutdown():
        turtle_pond.cancel_all_requests()
