#!/usr/bin/env python

import rospy

import csv
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch,ActionFeedback
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from coordinator.msg import MoveAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys

class Action:
    goto = "goto"
    land = "land"
    takeoff = "takeoff"
    follow = "follow"
    load = "load"
    unload = "unload"

class CoordinatorError(Exception):
     def __init__(self, value):
         self.value = value
     def __str__(self):
         return repr(self.value)


class Action:
    def __init__(self, action_dispatch_msg):
        self.action_id = action_dispatch_msg.action_id
        self.name = action_dispatch_msg.name

        self.parameters = action_dispatch_msg.parameters

        self.obj = None
        self.wp = None
        for p in self.parameters:
            if p.key == "obj":
                self.obj = p.value
            if p.key == "wp":
                self.wp = p.value

        if self.obj == None:
            raise CoordinatorError("No key obj in action dispatch parameter")
        if self.wp == None:
            raise CoordinatorError("No key wp in action dispatch parameter")


class Coordinator:
    def __init__(self, wp_file):
        self.waypoints = self.read_waypoints(wp_file)

        rospy.loginfo('/coordinator/__init__/ - Using waypoints from %s', wp_file)

        rospy.init_node('coordinator', anonymous=True, log_level=rospy.INFO)

        # Interface to ROSplan
        rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, self.action_dispatch_callback)

        # Set up Publisher
        self.feedback_pub = rospy.Publisher("/kcl_rosplan/action_feedback", ActionFeedback, queue_size=10)

        #Interface to Turtlebot
        self.turtle_move_ac = SimpleActionClient("TurtleMoveBaseAction", MoveBaseAction)
        rospy.sleep(0.1)

        #Interface to Bebop
        self.beebop_move_ac = SimpleActionClient("BebopMoveBaseAction", MoveBaseAction)
        rospy.sleep(0.1)


    def read_waypoints(self, filename):
        waypoints = None
        with open(filename, 'r') as csvfile:
            r = csv.reader(csvfile)
            waypoints = {w[0] : (float(w[1]),float(w[2])) for w in r}
        
        return waypoints


    def action_dispatch_callback(self, msg):
        # Check Action type and call correct functions.
        action = Action(msg)
        rospy.loginfo('/coordinator/action_dispatch_callback obj "%s", action "%s"', action.obj, action.name)

        if (msg.name == 'goto'):
            self.action_goto(action)
        elif (msg.name == 'takeoff'):
            self.action_takeoff(action)
        elif (msg.name == 'land'):
            self.action_land(action)
        elif (msg.name == 'load'):
            self.action_load(action)
        elif (msg.name == 'unload'):
            self.action_unload(action)
        elif (msg.name == 'follow'):
            self.action_follow(action)
        else:
            rospy.loginfo("No action called %s for obj %s", msg.name, msg.obj)


    def action_takeoff(self, action):
        pass


    def action_land(self, action):
        pass


    def action_load(self, action):
        pass


    def action_unload(self, action):
        pass


    def action_follow(self, action):
        pass


    def action_goto(self, action):
    	rospy.loginfo('/coordinator/action_goto for %s', action.obj)
        parameters = action.parameters
        action_id = action.action_id
        obj = action.obj
        wp = action.wp

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

        if not ac.wait_for_server(timeout=rospy.Duration(5)):
            rospy.loginfo("server timeout")

        # Notify action dispatcher of status
        feedback_msg = ActionFeedback(action_id = action_id, status = "action_enabled")
        self.feedback_pub.publish(feedback_msg)        

        ac.send_goal(goal)
        ac.wait_for_result()

        feedback_msg = ActionFeedback()
        feedback_msg.action_id = action_id
        if (ac.get_state() == GoalStatus.SUCCEEDED):
        	feedback_msg.status = "action_achieved"
        else:
        	feedback_msg.status = "action_failed"

        self.feedback_pub.publish(feedback_msg)


    def test_actions(self):
        dispatch_msg = ActionDispatch()
        dispatch_msg.name = "goto"
        dispatch_msg.parameters = [KeyValue('obj','turtle'), KeyValue('wp','wp1')]
        tmp_pub = rospy.Publisher("/kcl_rosplan/action_dispatch", ActionDispatch, queue_size=1)
        rospy.sleep(0.1)
        tmp_pub.publish(dispatch_msg)
        rospy.logdebug('coordinator:test_actions')


    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: coordinator.py waypoint_file")
    else:
        coordinator = Coordinator(sys.argv[1])
        rospy.sleep(0.1)
        coordinator.test_actions()
        coordinator.spin()
