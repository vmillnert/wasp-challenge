#!/usr/bin/env python

import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch,ActionFeedback


class CoordinatorListener:

    def __init__(self):

        rospy.init_node('coordinator_listener', anonymous=True, log_level=rospy.DEBUG)

        # Interface to ROSplan
        rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, self.action_dispatch_callback)
        rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, self.action_feedback_callback)

        rospy.spin()

    def action_dispatch_callback(self, msg):
        rospy.logdebug("coordinator_listener:action_dispatch_callback\n\tReceived a ActionDispatch message.")

    def action_feedback_callback(self, msg):
        rospy.logdebug("coordinator_listener:action_feedback_callback\n\tReceived a ActionFeedback message.")


if __name__ == '__main__':
    listener = CoordinatorListener()
