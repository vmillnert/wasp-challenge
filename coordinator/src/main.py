#!/usr/bin/env python

import rospy

import csv
from std_msgs.msg import String
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

        rospy.Publisher("/kcl_rosplan/action_feedback", ActionFeedback, queue_size=10)

        #Interface to Turtlebot
        ns_turtle = "turtlebot"
        turtle_ac = SimpleActionClient(ns_turtle, MoveAction)

        #Interface to Drone
        ns_drone = "beebop"

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def read_waypoints(self, filename):
        waypoints = None
        with open(filename, 'r') as csvfile:
            r = csv.reader(csvfile)
            waypoints = {w[0] : (w[1],w[2]) for w in r}
        
        return waypoints

    def action_dispatch_callback(self, msg):
        # Check Action type and call correct functions.
        if (msg.name == 'goto'):
            self.action_goto(msg.parameters)

    def action_goto(parameters):
        pass

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: coordinator.py waypoint_file")
    else:
        coordinator = Coordinator(sys.argv[1])
