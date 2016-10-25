#! /usr/bin/env python

import roslib
import rospy

import controller

from bebop_controller.msg import *
from actionlib import SimpleActionServer


class BebopActionServer(object):

    def __init__(self):
        # Create all the ActionServers and then start them.
        as_land = SimpleActionServer("BebopLandAction", BebopLandAction, execute_cb=self.cb_land, auto_start=False)
        as_load = SimpleActionServer("BebopLoadAction", BebopLoadAction, execute_cb=self.cb_load, auto_start=False)
        as_move = SimpleActionServer("BebopMoveBaseAction", BebopMoveBaseAction, execute_cb=self.cb_move_base, auto_start=False)
        as_takeoff = SimpleActionServer("BebopTakeOffAction", BebopTakeOffAction, execute_cb=self.cb_takeoff, auto_start=False)
        as_unload = SimpleActionServer("BebopUnloadAction", BebopUnloadAction, execute_cb=self.cb_unload, auto_start=False)
        as_follow = SimpleActionServer("BebopFollowAction", BebopFollowAction, execute_cb=self.cb_follow, auto_start=False)
        as_land.start()
        as_load.start()
        as_move.start()
        as_takeoff.start()
        as_unload.start()
        as_follow.start()

        rospy.loginfo("%s running", rospy.get_name())


    def spin(self):
        rospy.spin()


    def cb_load(self, goal):
        pass


    def cb_land(self, goal):
        pass


    def cb_follow(self, goal):
        pass


    def cb_unload(self, goal):
        pass


    def cb_takeoff(self, goal):
        pass


    def cb_move_base(self, goal):
        pass


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
