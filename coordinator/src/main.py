#!/usr/bin/env python

import rospy

import csv
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch,ActionFeedback
from actionlib import SimpleActionClient, SimpleGoalState
from coordinator.msg import MoveAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys

class CoordinatorError(Exception):
     def __init__(self, value):
         self.value = value
     def __str__(self):
         return repr(self.value)

class Coordinator:
    def __init__(self, wp_file):
        self.waypoints = self.read_waypoints(wp_file)

        rospy.init_node('coordinator', anonymous=True)

        # Interface to ROSplan
        rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, self.action_dispatch_callback)

        self.feedback_pub = rospy.Publisher("/kcl_rosplan/action_feedback", ActionFeedback, queue_size=10)

        #Interface to Turtlebot
        ns_turtle = "turtlebot"
        self.turtle_move_ac = SimpleActionClient(ns_turtle, MoveBaseAction)

        ns_drone = "beebop"
        self.beebop_move_ac = SimpleActionClient(ns_drone, MoveBaseAction)

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
        obj = None
        wp = None
        for p in parameters:
            if p.key == "obj":
                obj = p.value
            if p.key == "wp":
                wp = p.value
                
        if obj == None:
            raise CoordinatorError("No key obj in action dispatch parameter")
        if wp == None:
            raise CoordinatorError("No key wp in action dispatch parameter")


        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
  
        goal.target_pose.pose.position.x = self.waypoints[wp][0]
        goal.target_pose.pose.position.y = self.waypoints[wp][1]

        ac = None
        if obj == "drone":
            ac = self.beebop_move_ac
        elif obj == "turtle":
            ac = self.turtle_move_ac
        if ac == None:
            raise CoordinatorError("No action client exist for obj %s" % obj)

        ac.send_goal(goal)
        ac.wait_for_result()

        if (ac.get_state() == SimpleGoalState.DONE):
            print "Success!"
        else:
            print "Goal failed"

    def test_actions(self):
        dispatch_msg = ActionDispatch()
        dispatch_msg.name = "goto"
        dispatch_msg.parameters = [KeyValue('obj','drone'), KeyValue('wp','wp1')]
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
