#! /usr/bin/env python

import roslib
import rospy

from controller import Controller, ActionStatus
from geometry_msgs.msg import PointStamped

from bebop_controller.msg import *
from actionlib import SimpleActionServer


class BebopActionServer(object):

    def __init__(self):
        # Create all the ActionServers.
        self.as_land = SimpleActionServer("BebopLandAction", BebopLandAction, execute_cb=self.cb_land, auto_start=False)
        self.as_load = SimpleActionServer("BebopLoadAction", BebopLoadAction, execute_cb=self.cb_load, auto_start=False)
        self.as_move = SimpleActionServer("BebopMoveBaseAction", BebopMoveBaseAction, execute_cb=self.cb_move_base, auto_start=False)
        self.as_takeoff = SimpleActionServer("BebopTakeOffAction", BebopTakeOffAction, execute_cb=self.cb_takeoff, auto_start=False)
        self.as_unload = SimpleActionServer("BebopUnloadAction", BebopUnloadAction, execute_cb=self.cb_unload, auto_start=False)
        self.as_follow = SimpleActionServer("BebopFollowAction", BebopFollowAction, execute_cb=self.cb_follow, auto_start=False)

        # Frequency for controller feedback loop
        self.wait_rate = rospy.Rate(10)

        # Setup controller
        self.controller = Controller("bebop")
        self.controller.set_mode("auto")
        self.controller.run()

        # Finally, start action servers
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
        rospy.loginfo("/BebopActionServer/cb_load action_id %s", self.as_load.current_goal.get_goal_id().id)
        self.as_load.set_succeeded()


    def cb_land(self, goal):
        rospy.loginfo("/BebopActionServer/cb_land action_id %s", self.as_land.current_goal.get_goal_id().id)
        self.controller.land()
        self.handle_feedback(self.as_land)


    def cb_follow(self, goal):
        rospy.loginfo("/BebopActionServer/cb_follow action_id %s", self.as_follow.current_goal.get_goal_id().id)
        self.as_follow.set_succeeded()


    def cb_unload(self, goal):
        rospy.loginfo("/BebopActionServer/cb_unload action_id %s", self.as_unload.current_goal.get_goal_id().id)
        self.as_unload.set_succeeded()


    def cb_takeoff(self, goal):
        rospy.loginfo("/BebopActionServer/cb_takeoff action_id %s", self.as_takeoff.current_goal.get_goal_id().id)
        self.controller.takeoff()
        self.handle_feedback(self.as_takeoff)


    def cb_move_base(self, goal):
        rospy.loginfo("/BebopActionServer/cb_move_base action_id %s", self.as_move.current_goal.get_goal_id().id)
        point_goal = PointStamped()
        point_goal.header = goal.header
        point_goal.point = goal.target_pose.pose.position
        self.controller.setgoal(point_goal)
        self.handle_feedback(self.as_move)

    def handle_feedback(self, actionserver):
        preempted = False
        while self.controller.get_action_status() <= ActionStatus.STARTED:
            if rospy.is_shutdown():
                self.controller.abort_action()
                sys.exit()

            elif actionserver.is_preempt_requested():
                preempted = True
                self.controller.abort_action()

            self.rate.sleep()

        if self.controller.get_action_status() == ActionStatus.COMPLETED:
            actionserver.set_succeeded()
        else:
            if preempted:
                actionserver.set_preempted()
            else:
                actionserver.set_aborted()


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
