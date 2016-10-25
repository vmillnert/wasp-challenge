#! /usr/bin/env python

import roslib
import rospy

import controller

from bebop_controller.msg import *
from actionlib import SimpleActionServer


class BebopActionServer(object):

    def __init__(self):
        # Create all the ActionServers and then start them.
        self.as_land = SimpleActionServer("BebopLandAction", BebopLandAction, execute_cb=self.cb_land, auto_start=False)
        self.as_load = SimpleActionServer("BebopLoadAction", BebopLoadAction, execute_cb=self.cb_load, auto_start=False)
        self.as_move = SimpleActionServer("BebopMoveBaseAction", BebopMoveBaseAction, execute_cb=self.cb_move_base, auto_start=False)
        self.as_takeoff = SimpleActionServer("BebopTakeOffAction", BebopTakeOffAction, execute_cb=self.cb_takeoff, auto_start=False)
        self.as_unload = SimpleActionServer("BebopUnloadAction", BebopUnloadAction, execute_cb=self.cb_unload, auto_start=False)
        self.as_follow = SimpleActionServer("BebopFollowAction", BebopFollowAction, execute_cb=self.cb_follow, auto_start=False)
        self.as_land.start()
        self.as_load.start()
        self.as_move.start()
        self.as_takeoff.start()
        self.as_unload.start()
        self.as_follow.start()

        rospy.loginfo("%s running", rospy.get_name())


    def spin(self):
        rospy.spin()


    def cb_load(self, goal):
        rospy.loginfo("/BebopActionServer/cb_load action_id %f", goal.goal_id)
        self.as_load.set_succeeded()


    def cb_land(self, goal):
        rospy.loginfo("/BebopActionServer/cb_load action_id %f", goal.goal_id)
        self.as_land.set_succeeded()


    def cb_follow(self, goal):
        rospy.loginfo("/BebopActionServer/cb_load action_id %f", goal.goal_id)
        self.as_follow.set_succeeded()


    def cb_unload(self, goal):
        rospy.loginfo("/BebopActionServer/cb_load action_id %f", goal.goal_id)
        self.as_unload.set_succeeded()


    def cb_takeoff(self, goal):
        rospy.loginfo("/BebopActionServer/cb_load action_id %f", goal.goal_id)
        self.as_takeoff.set_succeeded()


    def cb_move_base(self, goal):
        rospy.loginfo("/BebopActionServer/cb_load action_id %f", goal.goal_id)
        self.as_move.set_succeeded()


if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('BebopActionServer', anonymous=True, log_level=rospy.INFO)
        # create the controller object
        bebop_as = BebopActionServer()
        rospy.sleep(0.1)
        bebop_as.spin()

    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
