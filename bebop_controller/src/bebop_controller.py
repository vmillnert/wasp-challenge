#! /usr/bin/env python


import roslib; roslib.load_manifest('bebop_controller')
import rospy
import numpy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String

class Controller(object):

    _MAX_VEL = 0.1 # Maximum velocity for the drone in any one
                   # direction
    _MAX_VELsmall = 0.1 # Maximum velocity for the drone in any one
                   # direction

                   
    _TOLERANCE = 0.5 # Tolerance for the Go-to-goal controller
    _TOLERANCEsmall = 0.3 # Tolerance for the Go-to-goal controller

    _P = 1 # P-parameter for the controller
    _Psmall = 0.1

    _goal_point = PointStamped() # the goal position expressed in a PointStamped
    _my_point = PointStamped()
    _error_point = PointStamped()

    _control_mode = 'manual' # Flag if the automatic controller should
                             # be running
    
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
        self._listener = tf.TransformListener()
        self._listener.waitForTransform('/base_link', '/odom',
                                        rospy.Time(0),
                                        rospy.Duration(5))

        # Set goal position here for now

        goal_point = PointStamped()
        goal_point.header.stamp = rospy.Time.now()
        goal_point.header.frame_id = 'odom'
        goal_point.point.x = -0.4
        goal_point.point.y = 0.0
        # rospy.sleep(2)

        self.convert_goal(goal_point)

        


        
    def teleop_command_callback(self, msg):
        # input commands from the bebop_teleop
        if msg.data == "takeoff":
            self.takeoff()
        elif msg.data == "land":
            self.land()
        elif msg.data == "manual":
            rospy.loginfo('%s: Swicthed to manual mode' % self._name)
            self._control_mode = 'manual'
        elif msg.data == "auto":
            rospy.loginfo('%s: Swicthed to automatic mode' % self._name)
            self._control_mode = 'auto'
        else:
            rospy.loginfo('%s: Got unknown command: %s \n', self._name, msg.data)


    def teleop_velocity_callback(self, msg):
        rospy.loginfo('%s: Teleop velocity: %s', self._name, msg)
        self._teleop_vel = msg

    def pos_callback(self, data):
        # Convert the collected PoseWithCovariance message into a PointStamped Message
        self._my_point.point = data.pose.pose.position
        self._my_point.header = data.header
        # rospy.loginfo('%s: Pos-data collected frame: %s',
        #               self._name, data.header.frame_id)

    def convert_goal(self, goal):
        # transform the goal-point to the odom-frame
        # goal = self._listener.transformPoint('odom', self._goal_point)
        self._goal_point = goal

    def compute_error(self):

        # Ideally we would like to convert our position into whathever
        # frame the goal_frame is expressed in

        # Step 1: retrieve the goal point, which now is expressed in 'odom' frame
        goal = self._goal_point

        # Step 2: compute the error between my current position
        # (expressed in odom-frame) and the goal position (also now
        # expressed in the odom-frame)

        # Right now this is done in the 'odom' frame

        xe = goal.point.x - self._my_point.point.x
        ye = goal.point.y - self._my_point.point.y
        ze = 0.0
        error_point = PointStamped()
        error_point.header = self._my_point.header
        error_point.point.x = xe
        error_point.point.y = ye
        error_point.point.z = ze
        
        # Step 3: convert it to the 'base_link'-frame
        try:
            error_point = self._listener.transformPoint('base_link',
                                                        error_point)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(e)
            pass
            
        
        return error_point
    
    def controller(self):
        loop_rate = rospy.Rate(30)

        
        while not rospy.is_shutdown():
            # self._goal_point.header.stamp = rospy.Time.now()
            P = self._P
            Psmall = self._Psmall
            
            # compute the error
            error_point = self.compute_error()
            rospy.loginfo('%s: The distance to goal in my_frame:\n x: %.2f \n y: %.2f',
                          self._name,
                          error_point.point.x, error_point.point.y)

            
            


            cmd_vel = Twist()
            if self._control_mode == 'manual':
                # rospy.loginfo('%s: We are in manual mode' % self._name)
                cmd_vel = self._teleop_vel
            elif self._control_mode == 'auto':
                # rospy.loginfo('%s: We are in automatic mode' % self._name)

                
                # We should do the automatic controller here
                xe = error_point.point.x
                ye = error_point.point.y

                # A simple P-controller here
                if numpy.sqrt(xe**2 + ye**2) > self._TOLERANCE:
                    # outside the tolerence, use the controller
                    vx = P*xe
                    vy = P*ye
                    # Limit the control-signal
                    if vx < -self._MAX_VEL:
                        vx = -self._MAX_VEL
                    elif vx > self._MAX_VEL:
                        vx = self._MAX_VEL
                    if vy < -self._MAX_VEL:
                        vy = -self._MAX_VEL
                    elif vy > self._MAX_VEL:
                        vy = self._MAX_VEL
                        
                    cmd_vel.linear.x = vx
                    cmd_vel.linear.y = vy
                elif numpy.sqrt(xe**2 + ye**2) > self._TOLERANCEsmall:
                    # outside the tolerence, use the controller
                    vx = Psmall*xe
                    vy = Psmall*ye
                    # Limit the control-signal
                    if vx < -self._MAX_VELsmall:
                        vx = -self._MAX_VELsmall
                    elif vx > self._MAX_VELsmall:
                        vx = self._MAX_VELsmall
                    if vy < -self._MAX_VELsmall:
                        vy = -self._MAX_VELsmall
                    elif vy > self._MAX_VELsmall:
                        vy = self._MAX_VELsmall
                        
                    cmd_vel.linear.x = vx
                    cmd_vel.linear.y = vy

                else:
                    # we arrived at the goal, should stand still here
                    rospy.loginfo('%s: We have arrived at the goal sir!' %self._name)
                    cmd_vel.linear.x = 0
                    cmd_vel.linear.y = 0
                    cmd_vel.linear.z = 0 
                    cmd_vel.angular.x = 0
                    cmd_vel.angular.y = 0
                    cmd_vel.angular.z = 0
                                       

            rospy.loginfo('%s: Velocity command sent: \n vx: %.2f \n vy: %.2f',
                          self._name,
                          cmd_vel.linear.x,
                          cmd_vel.linear.y)
            self._cmd_vel_pub.publish(cmd_vel)

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
        c.controller()
        # c.land()
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
