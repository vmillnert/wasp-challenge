#! /usr/bin/env python


import roslib; roslib.load_manifest('bebop_controller')
import rospy
from copy import deepcopy
import numpy
import tf

from PID import *
from PIDParameters import *
from yawPID import *

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_msgs.msg import String

class ActionStatus:
    IDLE = 0
    STARTED = 1
    FAILED = 2
    COMPLETED = 3

class Controller(object):

    _MAX_VEL = 0.4 # Maximum velocity for the drone in any one
                   # direction

    _TOLERANCE = 0.2 # Tolerance for the Go-to-goal controller
    _HEIGHT_TOL = 0.2 # tolerance for the height-controller
    _YAW_TOL = 0.2 # Tolerance for the yaw-controller
    
    def __init__(self, name):
        self._name = name

        self.action_status = ActionStatus.IDLE
        
        self.landing = False

        # The child frame for the drone
        self._child_frame_id = self._name + '/base_link'
        self._parent_frame_id = self._name + '/odom'

        # goal height for the bebop
        self._goal_height = 1.5 # initial height goal of the bebop

        self._goal_yaw = 0.0 # initial yaw-goal for the bebop
        
        # goal PointStamped ('odom'-frame)
        # only x & y position
        self._goal_point = PointStamped() 

        # Flag to know whether the goal is reached or not
        self.goal_reached = False

        # self._my_point = PointStamped() # current PointStamped ('odom'-frame)
        self._my_pose = PoseStamped() # current PoseStamed of the bebop ('odom'-frame)

        self._my_pose.header.frame_id = self._parent_frame_id
        self._my_pose.header.stamp = rospy.Time.now()
        self._teleop_vel = Twist() # control-signal sent by the
                                   # teleoperation keyboard


        # Control mode for the drone
        # 'manual' - from teleoperation
        # 'auto'   - go-to-goal behaviour using the PID-controller
        self._control_mode = 'manual' # start in 'manual'-mode


        ####################
        # Start subscribers

        # start the position callback
        rospy.Subscriber(self._name+'/odom', Odometry, self.pos_callback)

        # start the command callback from 'bebop_teleop'
        rospy.Subscriber('/bebop_teleop/command', String, self.teleop_command_callback)

        # start the vellocetiy callback from 'bebop_teleop'
        rospy.Subscriber('/bebop_teleop/cmd_vel', Twist, self.teleop_velocity_callback)

        # start the transformation listener
        # This will be moved up to the execute-controller once that
        # one is up and running
        self._listener = tf.TransformListener()
        # self._listener.waitForTransform(self._child_frame_id,
        #                                 self._parent_frame_id,
        #                                 rospy.Time(0),
        #                                 rospy.Duration(5))


        ###################
        # Start publishers
        self._takeoff_pub = rospy.Publisher(self._name+'/takeoff', Empty, queue_size=10)
        self._land_pub = rospy.Publisher(self._name+'/land', Empty, queue_size=10)
        self._cmd_vel_pub = rospy.Publisher(self._name+'/cmd_vel', Twist, queue_size=10)


        #################
        # # PID Controllers
        self.xPID = PID() # pid-controller for x-direction
        self.yPID = PID() # pid-controller for y-direction
        self.zPID = PID() # pid-controller for z-direction
        self.yawPID = yawPID() # pid-controller for the yaw (needs a special pid-controller due to the angles)

        ########################

        # Set goal position here for now
        # This is only used for takeoff from teleoperation
        # goal_point = PointStamped()
        # goal_point.header.stamp = rospy.Time.now()
        # goal_point.header.frame_id = self._parent_frame_id
        # goal_point.point.x = 0.0 # goal in x
        # goal_point.point.y = 0.0 # goal in y
        # self.set_goal(goal_point)


    # Set yaw goal of the bebop
    def set_yaw(self, yaw):
        self._goal_yaw = deepcopy(yaw)
        self.action_status = ActionStatus.STARTED
        rospy.sleep(1)
                
                
    # Set height goal of the Bebop
    def set_height(self, height):
        self._goal_height = deepcopy(height)
        self.action_status = ActionStatus.STARTED
        rospy.sleep(1)


    
        
    # Sets the goal position in 'odom'-frame
    # called by the goal_listener_callback()
    # Change so that it takes a
    # set_goal(point, height, yaw)
    # and that if height=0 it's not considered and similarily for the yaw
    def set_goal(self, goal):
        # check if it for some reason is not given in 'odom'-frame
        # if not goal.header.frame_id == self._my_point.header.frame_id:
        if not goal.header.frame_id == self._my_pose.header.frame_id:
        # convert is to odom:
            try:
                self._listener.waitForTransform(self._child_frame_id,
                                                goal.header.frame_id,
                                                goal.header.stamp,
                                                rospy.Duration(5))

                goal = self._listener.transformPoint(self._child_frame_id,
                                                     goal)
                self.action_status = ActionStatus.STARTED
                rospy.sleep(1)
                rospy.loginfo('%s: Got a new goal\n x: %.2f\n y: %.2f', self._name, goal.point.x, goal.point.y)
                self._goal_point = deepcopy(goal)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.loginfo(e)
                # Something went wrong, send 0 as control-signal
                # rospy.loginfo('%s: Could not convert goal from %s-frame to %s-frame',
                #               self._name,
                #               goal.header.frame_id,
                #               self._my_point.header.frame_id)
                rospy.loginfo('%s: Could not convert goal from %s-frame to %s-frame',
                              self._name,
                              goal.header.frame_id,
                              self._my_pose.header.frame_id)
                self.action_status = ActionStatus.FAILED            
                pass
            
        # Now we now that it is in the correct frame
        self.action_status = ActionStatus.STARTED
        rospy.sleep(1)
        rospy.loginfo('%s: Got a new goal\n x: %.2f\n y: %.2f', self._name, goal.point.x, goal.point.y)
        self._goal_point = deepcopy(goal)



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
        # self._my_point.point = data.pose.pose.position
        # self._my_point.header = data.header
        self._my_pose.header = data.header
        self._my_pose.pose = data.pose.pose
        self._child_frame_id = data.child_frame_id
        self._parent_frame_id = data.header.frame_id

        # rospy.loginfo('%s: child_frame_id: %s', self._name, self._child_frame_id)
        # rospy.loginfo('%s: parent_frame_id: %s', self._name, self._parent_frame_id)
        # rospy.loginfo('%s: Pos-data collected frame: %s',
        #               self._name, data.header.frame_id)


    # Takes the control-vector vx and vy (velocity commands) expressed
    # in 'odom'-frame and transforms it to 'base_link'-frame
    def convert_control_signal(self, vx, vy, vz, vyaw):
        # Try to transform the velocity-vector instead
        # Current position is stored in 'odom'-frame
        # control = PointStamped()
        # control = deepcopy(self._my_point)
        # control.point.x += deepcopy(vx)
        # control.point.y += deepcopy(vy)
        # control = Vector3Stamped()
        # control.header = deepcopy(self._my_point.header)
        # control.vector.x = deepcopy(vx)
        # control.vector.y = deepcopy(vy)
        # control.vector.z = deepcopy(vz)

        control = deepcopy(self._my_pose)
        control.pose.position.x += deepcopy(vx)
        control.pose.position.y += deepcopy(vy)
        control.pose.position.z += deepcopy(vz)
        control.pose.orientation.x,control.pose.orientation.y,control.pose.orientation.z,control.pose.orientation.w = tf.transformations.quaternion_from_euler(0.0, 0.0, vyaw)
        try:
            # control = self._listener.transformPoint('base_link',
            #                                         control)
            # control = self._listener.transformVector3('base_link', control)
            control = self._listener.transformPose(self._child_frame_id, control)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(e)
            # Something went wrong, send 0 as control-signal
            # control.point.x = 0.0
            # control.point.y = 0.0
            # control.point.z = 0.0
            # control.vector.x = 0.0
            # control.vector.y = 0.0
            # control.vector.z = 0.0
            control.pose = Pose()
            pass

        # return the x and y control-signals
        # return control.point.x, control.point.y
        # return control.vector.x, control.vector.y, control.vector.z
        return control.pose.position.x, control.pose.position.y, control.pose.position.z, tf.transformations.euler_from_quaternion([control.pose.orientation.x,control.pose.orientation.y,control.pose.orientation.z,control.pose.orientation.w])[2]


    # Make somebode know whether we have reached our goal or not
    def get_action_status(self):
        return deepcopy(self.action_status)

    # Abort current action goal
    # Set the goal-point to be the current position
    def abort_action(self):
        # Set the goal-point to be current point
        self._goal_point = PointStamped()
        self._goal_point.header = deepcopy(self._my_pose.header)
        self._goal_point.point = deepcopy(self._my_pose.pose.position)
        # set goal-height to be current height
        self._goal_height = deepcopy(self._my_pose.pose.position.z)
        # set goal-yaw to be current yaw
        self._goal_yaw = deepcopy(tf.transformations.euler_from_quaternion([self._my_pose.pose.orientation.x,
                                                                            self._my_pose.pose.orientation.y,
                                                                            self._my_pose.pose.orientation.z,
                                                                            self._my_pose.pose.orientation.w])[2])
        # reset the parameters
        self.xPID.reset()
        self.yPID.reset()
        self.zPID.reset()
        self.yawPID.reset()
        self.action_status = ActionStatus.FAILED

    
    # Run controller
    def run(self):

        # We should have a loop-period of self._H
        # which means we should have a loop-rate of 1/self._H
        loop_rate = rospy.Rate(1/self.xPID.p.H)


        while not rospy.is_shutdown():
            vx = 0.0
            vy = 0.0
            vz = 0.0
            vyaw = 0.0
            x = 0.0
            y = 0.0
            z = 0.0
            yaw = 0.0
            xref = 0.0
            yref = 0.0
            zref = 0.0
            yawref = 0.0
            cmd_vel = Twist()


            # Read goal position from the stored "_goal_point"
            # which is stored as PointStamped in frame 'odom'
            xref = deepcopy(self._goal_point.point.x)
            yref = deepcopy(self._goal_point.point.y)
            zref = deepcopy(self._goal_height)
            yawref = deepcopy(self._goal_yaw)
            
            # Read my current position from the stored "_my_point"
            # which is stored as PointStamped in frame 'odom'
            # x = deepcopy(self._my_point.point.x)
            # y = deepcopy(self._my_point.point.y)
            # z = deepcopy(self._my_point.point.z)

            x = deepcopy(self._my_pose.pose.position.x)
            y = deepcopy(self._my_pose.pose.position.y)
            z = deepcopy(self._my_pose.pose.position.z)
            yaw = deepcopy(tf.transformations.euler_from_quaternion([self._my_pose.pose.orientation.x,self._my_pose.pose.orientation.y,self._my_pose.pose.orientation.z,self._my_pose.pose.orientation.w])[2])
            
            if self._control_mode == 'manual':
                # Manual mode
                                  
                # the current action failed beacuse we went into manual mode
                if self.action_status == ActionStatus.STARTED:
                    self.action_status = ActionStatus.FAILED

                # Commands are sent via the teleop-controller

                # rospy.loginfo('%s: We are in manual mode' % self._name)

                cmd_vel = deepcopy(self._teleop_vel)

                ######################################
                # send control signal                #
                self._cmd_vel_pub.publish(cmd_vel) #
                ######################################

                # inform the user of control-velocities about to be sent
                # rospy.loginfo('%s: Velocity command sent: \n vx: %.2f \n vy: %.2f \n vz: %.2f\n vyaw: %.2f',
                #               self._name,
                #               cmd_vel.linear.x,
                #               cmd_vel.linear.y,
                #               cmd_vel.linear.z,
                #               cmd_vel.angular.z)

                # self._teleop_counter = self._teleop_counter + 1
                # if self._teleop_counter >= self._teleop_time:
                #     # set the command to 0
                #     self._teleop_vel = Twist()
                #     self._teleop_counter = 0

                                  

            # check if we are in automatic-mode or if the action has been aborted
            elif self._control_mode == 'auto' and (not self.landing):
                                  
                # Automatic mode
                self.goal_reached = (numpy.sqrt((xref-x)**2 + (yref-y)**2) < self._TOLERANCE) \
                    and (numpy.abs(zref-z) <  self._HEIGHT_TOL) \
                    and (numpy.abs(numpy.arctan2(numpy.sin(yawref-yaw),numpy.cos(yawref-yaw))) < self._YAW_TOL)

                # Check if we are withing our tolerance:
                # if (numpy.sqrt((xref-x)**2 + (yref-y)**2) > self._TOLERANCE) \
                #     or (numpy.abs(zref-z) >  self._HEIGHT_TOL) \
                #     or (numpy.abs(numpy.arctan2(numpy.sin(yawref-yaw),numpy.cos(yawref-yaw))) > self._YAW_TOL):
                if not self.goal_reached:
                    # Have some distance to the goal

                    # PID-control to make the bebop go to the desired
                    # goal-position

                    # rospy.loginfo('%s: We are in automatic mode' % self._name)

                    # Compute the control-signal ('odom'-frame)
                    # also stores the control-signal in self._vx and self._vy
                    vx = self.xPID.calculate_output(x, xref)
                    vy = self.yPID.calculate_output(y, yref)
                    vz = self.zPID.calculate_output(z, zref)
                    vyaw = self.yawPID.calculate_output(yaw, yawref)
                    
                    # limit the control-signals ('odom'-frame)
                    ux = self.limit(vx)
                    uy = self.limit(vy)
                    uz = self.limit(vz)
                    uyaw = self.limit(vyaw)

                    # convert the control signal from 'odom'-frame to
                    # 'base_link'-frame
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, cmd_vel.angular.z = self.convert_control_signal(ux,uy,uz,uyaw)

                    ######################################
                    # send control signal                #
                    self._cmd_vel_pub.publish(cmd_vel) #
                    ######################################

                    # inform the user of control-velocities about to be sent
                    # rospy.loginfo('%s: \n Distance to goal\n x: %.2f\n y: %.2f\n z: %.2f\n yaw: %.2f\n Velocity in [odom]:\n ux: %.2f\n uy: %.2f\n uz: %.2f\n yaw: %.2f \n Velocity command sent: \n vx: %.2f \n vy: %.2f\n vz: %.2f\n yaw: %.2f\n',
                    #               self._name,
                    #               xref-x,
                    #               yref-y,
                    #               zref-z,
                    #               yawref-yaw,
                    #               ux,
                    #               uy,
                    #               uz,
                    #               uyaw,
                    #               cmd_vel.linear.x,
                    #               cmd_vel.linear.y,
                    #               cmd_vel.linear.z,
                    #               cmd_vel.angular.z)

                    # update the state

                    self.xPID.update_state(ux)
                    self.yPID.update_state(uy)
                    self.zPID.update_state(uz)
                    self.yawPID.update_state(uyaw)

                else:
                    # We are within the tolerance for the goal!
                    rospy.loginfo('%s: We have arrived at the goal sir!',
                                  self._name)

                    if self.action_status == ActionStatus.STARTED:
                        self.action_status = ActionStatus.COMPLETED


                    # reset the PID controllers
                    self.xPID.reset()
                    self.yPID.reset()
                    self.yawPID.reset()
                    cmd_vel = Twist()

                    ######################################
                    # send control signal                #
                    self._cmd_vel_pub.publish(cmd_vel)   #
                    ######################################

                    # inform the user of control-velocities about to be sent
                    # rospy.loginfo('%s: \n Distance to goal\n x: %.2f\n y: %.2f\n z: %.2f\n yaw: %.2f \n Velocity command sent: \n vx: %.2f \n vy: %.2f\n vz: %.2f\n yaw: %.2f\n',
                    #               self._name,
                    #               xref-x,
                    #               yref-y,
                    #               zref-z,
                    #               yawref-yaw,
                    #               cmd_vel.linear.x,
                    #               cmd_vel.linear.y,
                    #               cmd_vel.linear.z,
                    #               cmd_vel.angular.z)
                


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
    # called from the Bebop Action Server
    def takeoff(self):
        rospy.loginfo('%s: Drone is taking off \n' % self._name)
        msg = Empty()
        rospy.sleep(0.5)
        self._goal_point = PointStamped()
        self._goal_point.header = deepcopy(self._my_pose.header)
        self._goal_point.point = deepcopy(self._my_pose.pose.position)
        # set goal-height to be current height
        self._goal_height = 1.5
        # set goal-yaw to be current yaw
        self._goal_yaw = deepcopy(tf.transformations.euler_from_quaternion([self._my_pose.pose.orientation.x,
                                                                            self._my_pose.pose.orientation.y,
                                                                            self._my_pose.pose.orientation.z,
                                                                            self._my_pose.pose.orientation.w])[2])
        # reset the parameters
        self.xPID.reset()
        self.yPID.reset()
        self.zPID.reset()
        self.yawPID.reset()
        self.action_status = ActionStatus.STARTED
        self.landing = False
        ######################################
        # send takeoff message               #
        self._takeoff_pub.publish(Empty())   #
        ######################################

    # Make the drone land
    # called from the Bebop Action Server
    # Should use Ardrone3PilotingStateFlyingStateChanged to determine 
    # if it has landed or not
    def land(self):
        rospy.loginfo('%s: Drone is landing \n' % self._name)
        msg = Empty()
        self.landing = True
        rospy.sleep(0.5)
        ######################################
        # send takeoff message               #
        self._land_pub.publish(Empty())      #
        ######################################
                                  
    
    # set the mode of the drone
    # called from the Bebop Action Server
    # mode: 'auto' - automatic mode
    # mode: 'manual' - manual mode
    def set_mode(self, mode):
        if mode == 'auto':
            rospy.loginfo('%s: Swicthed to %s mode', self._name, mode)
            self._control_mode = mode
        elif mode == 'manual':
            rospy.loginfo('%s: Swicthed to %s mode', self._name, mode)
            self._control_mode = mode
        

    # returns the current mode
    # called from the Bebop Action Server
    def get_action_status(self):
        return deepcopy(self.action_status)


if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('bebop')
        # create the controller object
        c = Controller(rospy.get_name())
        # start the controller
        c.run()
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
