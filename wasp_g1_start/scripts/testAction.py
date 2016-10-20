#! /usr/bin/env python

import roslib
roslib.load_manifest('move_base_msgs')
import rospy
import actionlib
from move_base_msgs.msg import *


if __name__ == '__main__':
    rospy.init_node('move_base_client')
    client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map" 
    goal.target_pose.header.stamp = rospy.Time.now() 
    goal.target_pose.pose.position.x = float(sys.argv[1])
    goal.target_pose.pose.position.y = float(sys.argv[2])
    goal.target_pose.pose.orientation.w = 1 
    client.send_goal(goal)
    res = client.wait_for_result()
    print(res)
