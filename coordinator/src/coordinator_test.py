#!/usr/bin/env python

import rospy
import roslib
# import roslib.load_manifest('coordinator')

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from rosplan_dispatch_msgs.msg import ActionDispatch,ActionFeedback
from geometry_msgs.msg import PoseStamped


class CoordinatorListener:

    def __init__(self):

        self._name = 'TurtleMoveBaseAction'

        rospy.init_node('coordinator_listener', anonymous=True, log_level=rospy.INFO)

        self.as_turtle = actionlib.SimpleActionServer(self._name, MoveBaseAction, execute_cb=self.execute_cb_turtle, auto_start = False)
        self.as_turtle.start()

        # Interface to ROSplan
        rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, self.action_dispatch_callback)
        rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, self.action_feedback_callback)

        rospy.spin()

    def action_dispatch_callback(self, msg):
        rospy.loginfo("coordinator_listener:action_dispatch_callback\n\tReceived a ActionDispatch message.")

    def action_feedback_callback(self, msg):
        rospy.loginfo("coordinator_listener:action_feedback_callback\n\tReceived a ActionFeedback message.")

    def execute_cb_turtle(self, goal):
        rospy.loginfo("coordinator_listener:execute_cb_turtle\n\tExecuting goal.")
        feedback_msg = MoveBaseFeedback()
        feedback_msg.base_position = PoseStamped()
        self.as_turtle.publish_feedback(feedback_msg)

        self.as_turtle.set_succeeded(MoveBaseResult())


if __name__ == '__main__':
    listener = CoordinatorListener()
