#! /usr/bin/env python


import roslib; roslib.load_manifest('bebop_controller')
import rospy
import numpy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String

class Controller(object):
    _pos = Point()
    _goal_pos = Point()

    _takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
    _land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    _cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
    _teleop_vel = Twist()

    def __init__(self, name):
        self._name = name        
        # start the position callback
        rospy.Subscriber('/bebop/odom', Odometry, self.pos_callback)
        # start the command callback from 'bebop_teleop'
        rospy.Subscriber('/bebop_teleop/command', String, self.teleop_command_callback)
        # start the vellocetiy callback from 'bebop_teleop'
        rospy.Subscriber('/bebop_teleop/cmd_vel', Twist, self.teleop_velocity_callback)
        # Add capabilities for an action server here
        # start the action server
        # Set initial goal position
        _goal_pos.x = 1.0
        _goal_pos.y = 0.0

        
    def teleop_command_callback(self, msg):
        # input commands from the bebop_teleop
        if msg.data == "takeoff":
            self.takeoff()
        elif msg.data == "land":
            self.land()
        else:
            rospy.loginfo('%s: Got unknown command: %s \n', self._name, msg.data)

    def teleop_velocity_callback(self, msg):
        rospy.loginfo('%s: Teleop velocity: %s', self._name, msg)
        self._teleop_vel = msg

    def pos_callback(self, data):
        self._pos = data.pose.pose.position
        # rospy.loginfo('%s: Updated position-data' % self._name)
    
    def run_controller(self):
        loop_rate = rospy.Rate(20)
        while not rospy.is_shutdown():
           rospy.loginfo('%s: Current pos:\n x: %d\n y: %d\n z: %d',  self._name,  self._pos.x, self._pos.y, self._pos.z) 
           rospy.loginfo('%s: Goal pos:\n x: %d\n y: %d\n z: %d', self._name, self._goal_pos.x, self._goal_pos.y, self._goal_pos.y)
            # Publish the velocity command sent by the bebop_teleop
            self._cmd_vel_pub.publish(self._teleop_vel)
            
            loop_rate.sleep()
        
    def takeoff(self):
        rospy.loginfo('%s: Drone is taking off \n' % self._name)
        msg = Empty()
        rospy.sleep(0.5)
        self._takeoff_pub.publish(Empty())

    def land(self):
        rospy.loginfo('%s: Drone is landing \n' % self._name)
        msg = Empty()
        rospy.sleep(0.5)
        self._land_pub.publish(Empty())


if __name__ == '__main__':
    
    try:
        # takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10) #
        # rospy.sleep(2)
        # land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        # rospy.sleep(2)
        rospy.init_node('bebop_controller')
        c = Controller(rospy.get_name())
        # c.takeoff()
        # rospy.sleep(5)
        c.run_controller()
        # c.land()
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
