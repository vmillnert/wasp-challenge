#!/usr/bin/env python

import rospy

import csv
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch,ActionFeedback
from actionlib import SimpleActionClient
from coordinator.msg import MoveAction
import sys


class Coordinator:
    def __init__(self, wp_file):
        self.waypoints = self.read_waypoints(wp_file)

        rospy.init_node('coordinator', anonymous=True)

        # Interface to ROSplan
        rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, self.action_dispatch_callback)

        self.feedback_pub = rospy.Publisher("/kcl_rosplan/action_feedback", ActionFeedback, queue_size=10)

        #Interface to Turtlebot
        ns_turtle = "turtlebot"
        self.turtle_ac = SimpleActionClient(ns_turtle, MoveAction)

        ns_drone = "beebop"
        self.beebop_ac = SimpleActionClient(ns_drone, MoveAction)

    def read_waypoints(self, filename):
        waypoints = None
        with open(filename, 'r') as csvfile:
            r = csv.reader(csvfile)
            waypoints = {w[0] : (w[1],w[2]) for w in r}
        
        return waypoints

    def action_dispatch_callback(self, msg):
        # Check Action type and call correct functions.
        print "In dispatch feedback"
        if (msg.name == 'goto'):
            self.action_goto(msg.parameters)

    def action_goto(self, parameters):
        print "in goto"

    def test_actions(self):
        dispatch_msg = ActionDispatch()
        dispatch_msg.name = "goto"
        dispatch_msg.parameters = [KeyValue('obj','d0'), KeyValue('wp','wp1')]
        tmp_pub = rospy.Publisher("/kcl_rosplan/action_dispatch", ActionDispatch, queue_size=10)
        rospy.sleep(0.1)
        tmp_pub.publish(dispatch_msg)
        print "Publishing"
            
    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: coordinator.py waypoint_file")
    else:
        coordinator = Coordinator(sys.argv[1])
        coordinator.test_actions()
        coordinator.spin()
