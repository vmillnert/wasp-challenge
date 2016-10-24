#! /usr/bin/env python


import roslib; roslib.load_manifest('bebop_controller')
import rospy
from copy import deepcopy
import numpy
import tf

from PID import *
from PIDParameters import *

from bebop_controller.msg import *

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String

class Controller(object):

    _MAX_VEL = 0.3 # Maximum velocity for the drone in any one
                   # direction
                   
    _TOLERANCE = 0.2 # Tolerance for the Go-to-goal controller
    
    def __init__(self, name):
        self._name = name

        self._goal_point = PointStamped() # goal PointStamped ('odom'-frame)
        self._my_point = PointStamped() # current PointStamped ('odom'-frame)
        self._teleop_vel = Twist() # control-signal sent by the
                              # teleoperation keyboard

                              
        # Control mode for the drone
        # 'manual' - from teleoperation
        # 'auto'   - go-to-goal behaviour using the PID-controller
        self._control_mode = 'manual' # start in 'manual'-mode




        ####################
        # Start subscribers
        
        # start the position callback
        rospy.Subscriber('/bebop/odom', Odometry, self.pos_callback)
        
        # start the command callback from 'bebop_teleop'
        rospy.Subscriber('/bebop_teleop/command', String, self.teleop_command_callback)
        
        # start the vellocetiy callback from 'bebop_teleop'
        rospy.Subscriber('/bebop_teleop/cmd_vel', Twist, self.teleop_velocity_callback)

        # start the transformation listener
        # This will be moved up to the execute-controller once that
        # one is up and running
        self._listener = tf.TransformListener()
        self._listener.waitForTransform('/base_link', '/odom',
                                        rospy.Time(0),
                                        rospy.Duration(5))


        ###################
        # Start publishers
        self._takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self._land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        self._cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        
        #################
        # # PID Controllers
        self.xPID = PID() # pid-controller for x-direction
        self.yPID = PID() # pid-controller for y-direction

        # Note: when actually sending the command signal it has to be
        # transformed into 'base_link'-frame before beingn sent

        ########################

        # Set goal position here for now
        goal_point = PointStamped()
        goal_point.header.stamp = rospy.Time.now()
        goal_point.header.frame_id = 'odom'
        goal_point.point.x = 0.0
        goal_point.point.y = 0.0
        self.set_goal(goal_point)


        
    # Sets the goal position in 'odom'-frame
    # called by the goal_listener_callback()
    def set_goal(self, goal):

        # check if it for some reason is not given in 'odom'-frame
        if not goal.header.frame_id == self._my_point.header.frame_id:
            # convert is to odom:
            try:
                goal = self._listener.transformPoint('base_link',
                                                        control)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.loginfo(e)
                # Something went wrong, send 0 as control-signal
                rospy.loginfo('%s: Could not convert goal from %s-frame to %s-frame',
                              self._name,
                              goal.header.frame_id,
                              self._my_point.header.frame_id)
                pass

        # Now we are sure it is in the correct frame
        self._goal_point = goal




    # Limit the control signal v by _MAX_VEL 
    def limit(self, v):
        if v < -self._MAX_VEL:
            v = -self._MAX_VEL
        elif v > self._MAX_VEL:
            v = self._MAX_VEL

        return v

    # Reads velocity commands sent from the teleoperation-keyboard
    def teleop_velocity_callback(self, msg):
        # rospy.loginfo('%s: Teleop velocity: %s', self._name, msg)
        self._teleop_vel = msg
        self._teleop_counter = 0


    # Reads and stores the current position of the bebop
    def pos_callback(self, data):
        # Convert the collected PoseWithCovariance message into a
        # PointStamped message
        self._my_point.point = data.pose.pose.position
        self._my_point.header = data.header
        # rospy.loginfo('%s: Pos-data collected frame: %s',
        #               self._name, data.header.frame_id)


    # Takes the control signal vx and vy (velocity commands) expressed
    # in 'odom'-frame and transforms it to 'base_link'-frame
    def convert_control_signal(self, vx, vy):
        # Current position is stored in 'odom'-frame
        control = PointStamped()
        control = deepcopy(self._my_point)
        control.point.x += deepcopy(vx)
        control.point.y += deepcopy(vy)
        try:
            control = self._listener.transformPoint('base_link',
                                                    control)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(e)
            # Something went wrong, send 0 as control-signal
            control.point.x = 0.0
            control.point.y = 0.0
            control.point.z = 0.0
            pass

        # return the x and y control-signals
        return control.point.x, control.point.y


    # Run controller
    def run(self):
        
        # We should have a loop-period of self._H
        # which means we should have a loop-rate of 1/self._H
        loop_rate = rospy.Rate(1/self.xPID.p.H)

        
        while not rospy.is_shutdown():
            vx = 0.0
            vy = 0.0
            x = 0.0
            y = 0.0
            xref = 0.0
            yref = 0.0
            
            cmd_vel = Twist()


            # Read goal position from the stored "_goal_point"
            # which is stored as PointStamped in frame 'odom'
            xref = deepcopy(self._goal_point.point.x)
            yref = deepcopy(self._goal_point.point.y)
            
            # Read my current position from the stored "_my_point"
            # which is stored as PointStamped in frame 'odom'
            x = deepcopy(self._my_point.point.x)
            y = deepcopy(self._my_point.point.y)

            
            if self._control_mode == 'manual':
                
                # Manual mode

                # Commands are sent via the teleop-controller
                
                # rospy.loginfo('%s: We are in manual mode' % self._name)
                
                cmd_vel = deepcopy(self._teleop_vel)

                ######################################
                # send control signal                #
                self._cmd_vel_pub.publish(cmd_vel) #
                ######################################

                # inform the user of control-velocities about to be sent
                rospy.loginfo('%s: Velocity command sent: \n vx: %.2f \n vy: %.2f \n th: %.2f',
                              self._name,
                              cmd_vel.linear.x,
                              cmd_vel.linear.y,
                              cmd_vel.angular.z)

                # self._teleop_counter = self._teleop_counter + 1 
                # if self._teleop_counter >= self._teleop_time:
                #     # set the command to 0
                #     self._teleop_vel = Twist()
                #     self._teleop_counter = 0
                    
                
            elif self._control_mode == 'auto':

                # Automatic mode

                # Check if we are withing our tolerance:
                if numpy.sqrt((xref-x)**2 + (yref-y)**2) > self._TOLERANCE:
                    # Have some distance to the goal
                    
                    # PID-control to make the bebop go to the desired
                    # goal-position
                
                    # rospy.loginfo('%s: We are in automatic mode' % self._name)

                    # Compute the control-signal ('odom'-frame)
                    # also stores the control-signal in self._vx and self._vy
                    vx = self.xPID.calculate_output(x, xref)
                    vy = self.yPID.calculate_output(y, yref)
                    
                    # limit the control-signals ('odom'-frame)
                    ux = self.limit(vx)
                    uy = self.limit(vy)

                    
                    # convert the control signal from 'odom'-frame to
                    # 'base_link'-frame
                    cmd_vel.linear.x, cmd_vel.linear.y = self.convert_control_signal(ux, uy)

                    ######################################
                    # send control signal                #
                    self._cmd_vel_pub.publish(cmd_vel) #
                    ######################################
                
                    # inform the user of control-velocities about to be sent
                    rospy.loginfo('%s: \n Distance to goal\n x: %.2f\n y: %.2f\n Velocity in [odom]:\n ux: %.2f\n uy: %.2f\n Velocity command sent: \n vx: %.2f \n vy: %.2f',
                                  self._name,
                                  xref-x,
                                  yref-y,
                                  ux,
                                  uy,
                                  cmd_vel.linear.x,
                                  cmd_vel.linear.y)

                    # update the state

                    self.xPID.update_state(ux)
                    self.yPID.update_state(uy)

                else:
                    # We are within the tolerance for the goal!
                    rospy.loginfo('%s: We have arrived at the goal sir!',
                                  self._name)
                    
                    # reset the PID controllers
                    self.xPID.reset()
                    self.yPID.reset()
                    cmd_vel = Twist()

                    ######################################
                    # send control signal                #
                    self._cmd_vel_pub.publish(cmd_vel) #
                    ######################################
                
                    # inform the user of control-velocities about to be sent
                    rospy.loginfo('%s:\n My pos:\n x: %.2f\n y: %.2f \n Distance to goal\n x: %.2f\n y: %.2f\n Velocity command sent: \n vx: %.2f \n vy: %.2f',
                                  self._name,
                                  x,
                                  y,
                                  x-xref,
                                  y-yref,
                                  cmd_vel.linear.x,
                                  cmd_vel.linear.y)


            loop_rate.sleep()


    # Commands from the teleoperation keyboard
    def teleop_command_callback(self, msg):
        # input commands from the bebop_teleop
        if msg.data == "takeoff":
            self.takeoff()
            # reset the PID-controller
            self.xPID.reset()
            self.yPID.reset() 
        elif msg.data == "land":
            self.land()
            # reset the PID-controller
            self.xPID.reset()
            self.yPID.reset() 
        elif msg.data == "manual":
            rospy.loginfo('%s: Swicthed to manual mode' % self._name)
            self._control_mode = 'manual'
            # reset the PID-controller
            self.xPID.reset()
            self.yPID.reset() 
        elif msg.data == "auto":
            rospy.loginfo('%s: Swicthed to automatic mode' % self._name)
            self._control_mode = 'auto'
            # reset the PID-controller
            self.xPID.reset()
            self.yPID.reset() 
        else:
            rospy.loginfo('%s: Got unknown command: %s \n', self._name, msg.data)


    # Make the drone takeoff
    def takeoff(self):
        rospy.loginfo('%s: Drone is taking off \n' % self._name)
        msg = Empty()
        rospy.sleep(0.5)
        ######################################
        # send takeoff message               #
        self._takeoff_pub.publish(Empty()) #
        ######################################

    # Make the drone land
    def land(self):
        rospy.loginfo('%s: Drone is landing \n' % self._name)
        msg = Empty()
        rospy.sleep(0.5)
        ######################################
        # send takeoff message               #
        self._land_pub.publish(Empty())    #
        ######################################


if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('bebop_controller')
        # create the controller object
        c = Controller(rospy.get_name())
        # start the controller
        c.run()
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
