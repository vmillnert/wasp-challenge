#! /usr/bin/env python


import roslib; roslib.load_manifest('bebop_controller')
import rospy
from copy import deepcopy
import numpy
import tf
from threading import RLock
import math

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
from ardrone_autonomy.msg import Navdata
from gazebo_msgs.msg import ModelStates

class ActionStatus:
    IDLE = 0
    STARTED = 1
    FAILED = 2
    COMPLETED = 3

ARDroneState = {
  'UNKNOWN': 0,
  'INITED':1,
  'LANDED': 2,
  'FLYING1': 3,
  'FLYING2': 7,
  'HOVERING': 4,
  'TEST': 5,
  'TAKING_OFF': 6,
  'LANDING': 8,
  'LOOPING': 9
}

class Action:
  NONE = 0
  TAKEOFF = 1
  LAND = 2
  GOTO = 3

class Controller:
  def __init__(self, name):
    self._name = name
    self.action_status = ActionStatus.IDLE
    self.action = Action.NONE
    self.lock = RLock()

    # Control mode for the drone
    # 'manual' - from teleoperation
    # 'auto'   - go-to-goal behaviour using the PID-controller
    self._control_mode = 'manual' # start in 'manual'-mode

    self.tfListener = tf.TransformListener()

  def set_yaw(self, yaw):
    pass

  def set_height(self, height):
    print("N.A. set_height")

  def set_goal(self, goal):
    print("N.A. set_goal")

  def limit(self, v):
    print("N.A. limit")

  def teleop_velocity_callback(self, msg):
    print("N.A. teleop_velocity_callback")

  def pos_callback(self, data):
    print("N.A. pos_callback")

  def convert_control_signal(self, vx, vy, vz, vyaw):
    print("N.A. convert_control_signal")

  # returns the current mode
  # called from the Bebop Action Server
  def get_action_status(self):
    self.lock.acquire()
    s = deepcopy(self.action_status)
    self.lock.release()
    return s

  def set_action_status(self, s):
    self.lock.acquire()
    self.action_status = deepcopy(s)
    self.lock.release()

  def abort_action(self):
    print("N.A. abort_action")

  def run(self):
    print("N.A. run")

  def teleop_command_callback(self, msg):
    print("N.A.teleop_command_callback")

  def takeoff(self):
    print("N.A. takeoff")

  def land(self):
    print("N.A. land")

  def set_mode(self, mode):
    self.lock.acquire()
    if mode == 'auto':
      rospy.loginfo('%s: Swicthed to %s mode', self._name, mode)
      self._control_mode = mode
    elif mode == 'manual':
      rospy.loginfo('%s: Swicthed to %s mode', self._name, mode)
      self._control_mode = mode
    self.lock.release()

# Subtract pose p1-p2
def subtract_pose(p1, p2):
  d = Pose()
  d.position.x = p1.position.x-p2.position.x
  d.position.y = p1.position.y-p2.position.y
  d.position.z = p1.position.z-p2.position.z
  d.orientation.x = p1.orientation.x-p2.orientation.x
  d.orientation.y = p1.orientation.y-p2.orientation.y
  d.orientation.z = p1.orientation.z-p2.orientation.z
  d.orientation.w = p1.orientation.w-p2.orientation.w
  return d


# 2-norm of the pose position in x and y
def norm2d(p):
  x = p.position.x
  y = p.position.y
  return math.sqrt(x*x+y*y)

# 2-norm of 3d coordinate
def norm3d(p):
  x = p.position.x
  y = p.position.y
  z = p.position.z
  return math.sqrt(x*x+y*y+z*z)

# Maximum abs of x,y,z
def maxabs3d(p):
  return max(max(abs(p.position.x), abs(p.position.y)), abs(p.position.z))

# Get the yaw of a pose
def getyaw(p):
  q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
  e = tf.transformations.euler_from_quaternion(q)
  return e[2]


# Update the yaw of a pose
def setyaw(p, yaw):
  q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
  e = tf.transformations.euler_from_quaternion(q)
  q = tf.transformations.quaternion_from_euler(e[0], e[1], yaw)
  p.orientation.x = q[0]
  p.orientation.y = q[1]
  p.orientation.z = q[2]
  p.orientation.w = q[3]

# Translates and rotates a pose. The new pose is returned as (trans, rot)
def transform_pose(p, trans, orient):
  transmat = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(orient))
  me = tf.transformations.concatenate_matrices(
          tf.transformations.translation_matrix((
            p.position.x,
            p.position.y,
            p.position.z)),
          tf.transformations.quaternion_matrix((
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w
            )))
  posmat = numpy.dot(transmat, me)
  return (tf.transformations.translation_from_matrix(posmat),
          tf.transformations.quaternion_from_matrix(posmat))

class ARDroneSimController(Controller):
  def __init__(self, name):
    Controller.__init__(self, name)
    self.action_counter = -1
    self.rate = 50

    # Subscribers
    rospy.Subscriber('ardrone/navdata', Navdata, self.cb_navdata)
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_modelstates)

    # start the command callback from 'bebop_teleop' TODO: Base class!!
    rospy.Subscriber('bebop_teleop/command', String, self.teleop_command_callback)
    rospy.Subscriber('bebop_teleop/cmd_vel', Twist, self.teleop_velocity_callback)

    # Publishers
    self.pub_takeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
    self.pub_land = rospy.Publisher('ardrone/land', Empty, queue_size=10)
    self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10) # TODO!!! Namespace

    self.worldpose = Pose() # In the gazebo world
    self.pose = Pose()      # On the map
    self.target = Pose()
    self.velo = [0.0, 0.0, 0.0] # TODO: Listen to Navdata
    self.navdata = False

  # Transforms world position to map
  def update_pose(self):
    try:
      (trans, orient) = self.tfListener.lookupTransform("map", "world", rospy.Time(0))
      (t, o) = transform_pose(self.worldpose, trans, orient)
      self.pose.position.x = t[0]
      self.pose.position.y = t[1]
      self.pose.position.z = t[2]
      self.pose.orientation.x = o[0]
      self.pose.orientation.y = o[1]
      self.pose.orientation.z = o[2]
      self.pose.orientation.w = o[3]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print("EXCEPTION")

  def cb_navdata(self, data):
    self.lock.acquire();
    self.navdata = deepcopy(data)
    self.lock.release();

  def cb_modelstates(self, data):
    try:
      i = data.name.index(self._name)
      self.lock.acquire();
      self.worldpose = deepcopy(data.pose[i])
      self.lock.release();
    except:
      pass

  def airborne(self):
    self.lock.acquire();
    s = self.navdata.state
    self.lock.release();
    return s == ARDroneState['FLYING1'] or s == ARDroneState['FLYING2'] or s == ARDroneState['HOVERING'] or s == ARDroneState['LOOPING'];

  def grounded(self):
    self.lock.acquire();
    s = self.navdata.state
    self.lock.release();
    return s == ARDroneState['INITED'] or s == ARDroneState['LANDED']

  def printstate(self):
    print("{} is {} [{}]\n  pose [P({:4.1f},{:4.1f},{:4.1f}) R({:4.1f},{:4.1f},{:4.1f},{:4.1f}) Y({:4.3f})]\n  velocity ({:3.1f},{:3.1f},{:3.1f} [S({:3.1f},{:3.1f},{:3.1f})])".format(self._name,
      ARDroneState.keys()[ARDroneState.values().index(self.navdata.state)], self._control_mode,
      self.pose.position.x, self.pose.position.y, self.pose.position.z, 
      self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w,
      getyaw(self.pose),
      self.navdata.vx, self.navdata.vy, self.navdata.vz,
      self.velo[0], self.velo[1], self.velo[2]))
    if self.action == Action.GOTO:
      print("  target [P({:4.1f},{:4.1f},{:4.1f}) R({:4.1f},{:4.1f},{:4.1f},{:4.1f}) Y({:4.3f})]".format(
        self.target.position.x, self.target.position.y, self.target.position.z, 
        self.target.orientation.x, self.target.orientation.y, self.target.orientation.z, self.target.orientation.w,
        getyaw(self.target)))
    if not self.action == Action.NONE and self.get_action_status() == ActionStatus.STARTED:
      print("  acting")
 
  def run(self):
    # TODO: Move into base class
    printcounter = 0
    loop_rate = rospy.Rate(self.rate)

    # Wait for initial navdata
    self.lock.acquire()
    while not self.navdata and not rospy.is_shutdown():
      self.lock.release()
      loop_rate.sleep()
      self.lock.acquire()
    self.update_pose()
    self.target = deepcopy(self.pose)
    self.lock.release()

    # Init commands. Why is there velocity set even when zeroed?
    msg = Twist()
    msg.linear.x = msg.linear.y = msg.linear.z = 0
    self.pub_vel.publish(msg)
   
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.update_pose()
      printcounter += 1
      if printcounter == self.rate*5:
        self.printstate()
        printcounter = 0

      if self._control_mode == 'manual':
        self.manualmode()
      else:
        self.automode()
      self.lock.release()
      loop_rate.sleep()

  def manualmode(self):
    if self.get_action_status() == ActionStatus.STARTED:
      # the current action failed beacuse we went into manual mode
      self.set_action_status(ActionStatus.FAILED)
    cmd_vel = deepcopy(self._teleop_vel)
    self.pub_vel.publish(cmd_vel)
 
  def automode(self):
    if self.get_action_status() == ActionStatus.STARTED:
      if self.action == Action.TAKEOFF:
        if self.airborne():
          self.set_height(1.5) # Not done yet, rise to height 1.5 first
      elif self.action == Action.GOTO:
        e = subtract_pose(self.target, self.pose)
        ye = getyaw(self.target)-getyaw(self.pose)

        position_good = maxabs3d(e) < 0.2
        yaw_good = abs(ye) < 0.02

        velocity = (0,0,0)
        rotation = 0

        if not position_good:
          velocity = self.control_speed(e)

        if not yaw_good:
          rotation = self.control_rotation(ye)

        self.send_movement(velocity, rotation)

        if position_good and yaw_good:
          self.printstate()
          self.set_action_status(ActionStatus.COMPLETED)

      elif self.action == Action.LAND:
        if self.grounded():
          self.printstate()
          self.set_action_status(ActionStatus.COMPLETED)

  def takeoff(self):
    self.lock.acquire()
    self.pub_takeoff.publish(Empty())
    # TODO: An action object
    self.action = Action.TAKEOFF
    self.set_action_status(ActionStatus.STARTED)
    self.lock.release()

  def land(self):
    self.lock.acquire()
    self.pub_land.publish(Empty())
    self.action = Action.LAND
    self.set_action_status(ActionStatus.STARTED)
    self.lock.release()

  def set_yaw(self, yaw):
    self.lock.acquire()
    setyaw(self.target, yaw)
    self.action = Action.GOTO
    self.set_action_status(ActionStatus.STARTED)
    self.printstate()
    self.lock.release()

  def set_height(self, height):
    self.lock.acquire()
    self.target.position.z = height
    self.action = Action.GOTO
    self.set_action_status(ActionStatus.STARTED)
    self.printstate()
    self.lock.release()

  def set_goal(self, goal):
    if goal.header.frame_id != 'map':
      print("BAD FRAME!!!") # TODO Convert
    self.lock.acquire()
    self.action = Action.GOTO
    self.set_action_status(ActionStatus.STARTED)

    self.target.position.x = goal.point.x
    self.target.position.y = goal.point.y

    target_yaw = getyaw(self.target)
    self.target.orientation = deepcopy(self.pose.orientation)
    setyaw(self.target, target_yaw)
    
    self.lock.release()

  # Returns desired yaw rotation
  def control_rotation(self, yaw_err):
    return 250*yaw_err/self.rate;

  # Returns desired velocities
  def control_speed(self, err):
    e = (err.position.x, err.position.y, err.position.z)
    for i in range(0,3): # x,y only
      v = max(-1, min(1, 100*e[i]/self.rate))
      self.velo[i] = v
    return (self.velo[0], self.velo[1], self.velo[2])

  def send_movement(self, velocities, rotation):
    msg = Twist()
    msg.linear.x = velocities[0]
    msg.linear.y = velocities[1]
    msg.linear.z = velocities[2]

    msg.angular.z = rotation
    self.pub_vel.publish(msg)

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

  # Reads velocity commands sent from the teleoperation-keyboard
  def teleop_velocity_callback(self, msg):
    self._teleop_vel = msg
    self._teleop_counter = 0


class BebopController(Controller):

    _MAX_VEL = 0.4 # Maximum velocity for the drone in any one
                   # direction

    _TOLERANCE = 0.2 # Tolerance for the Go-to-goal controller
    _HEIGHT_TOL = 0.2 # tolerance for the height-controller
    _YAW_TOL = 0.2 # Tolerance for the yaw-controller
    
    def __init__(self, name):
        Controller.__init__(self, name)

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


        ####################
        # Start subscribers

        # start the position callback
        rospy.Subscriber(self._name+'/odom', Odometry, self.pos_callback)

        # start the command callback from 'bebop_teleop'
        rospy.Subscriber('bebop_teleop/command', String, self.teleop_command_callback)

        # start the vellocetiy callback from 'bebop_teleop'
        rospy.Subscriber('bebop_teleop/cmd_vel', Twist, self.teleop_velocity_callback)

       # self.tfListener.waitForTransform(self._child_frame_id,
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
        self.set_action_status(ActionStatus.STARTED)
        rospy.sleep(1)
                
                
    # Set height goal of the Bebop
    def set_height(self, height):
        self._goal_height = deepcopy(height)
        self.set_action_status(ActionStatus.STARTED)
        rospy.sleep(1)


    
        
    # Sets the goal position in 'odom'-frame
    # called by the goaltfListener_callback()
    # Change so that it takes a
    # set_goal(point, height, yaw)
    # and that if height=0 it's not considered and similarily for the yaw
    def set_goal(self, goal):
        # check if it for some reason is not given in 'odom'-frame
        # if not goal.header.frame_id == self._my_point.header.frame_id:
        if not goal.header.frame_id == self._my_pose.header.frame_id:
        # convert is to odom:
            try:
                self.tfListener.waitForTransform(self._child_frame_id,
                                                goal.header.frame_id,
                                                goal.header.stamp,
                                                rospy.Duration(5))

                goal = self.tfListener.transformPoint(self._child_frame_id,
                                                     goal)
                self.set_action_status(ActionStatus.STARTED)
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
                self.set_action_status(ActionStatus.FAILED)
                pass
            
        # Now we now that it is in the correct frame
        self._goal_point = deepcopy(goal)
        self.set_action_status(ActionStatus.STARTED)
        rospy.loginfo('%s: Got a new goal\n x: %.2f\n y: %.2f', self._name, goal.point.x, goal.point.y)
        rospy.loginfo('%s: Action_status: %d\n', self._name, self.get_action_status())
        # rospy.sleep(1)




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
            # control = self.tfListener.transformPoint('base_link',
            #                                         control)
            # control = self.tfListener.transformVector3('base_link', control)
            control = self.tfListener.transformPose(self._child_frame_id, control)
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
        self.set_action_status(ActionStatus.FAILED)

    
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
            action_status = self.get_action_status()

            if self._control_mode == 'manual':
                # Manual mode
                                  
                # the current action failed beacuse we went into manual mode
                if action_status == ActionStatus.STARTED:
                    self.set_action_status(ActionStatus.FAILED)

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


            # check if we are in automatic-mode or if the action has been aborted
            elif self._control_mode == 'auto' and (not self.landing):
                rospy.loginfo('%s: \n self.action_status: %d\n action_status: %d \n', 
                              self._name, self.get_action_status(), action_status)
                                  
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


                    # update the state

                    self.xPID.update_state(ux)
                    self.yPID.update_state(uy)
                    self.zPID.update_state(uz)
                    self.yawPID.update_state(uyaw)

                else:
                    # We are within the tolerance for the goal!
                    rospy.loginfo('%s: \n self.action_status: %d\n action_status: %d \n', 
                                  self._name, self.get_action_status(), action_status)

                    if action_status == ActionStatus.STARTED:
                        rospy.loginfo('%s: \n action_status==STARTED \n self.action_status: %d\n action_status: %d \n', 
                                      self._name, self.get_action_status(), action_status)

                        if action_status==self.get_action_status():
                            self.set_action_status(ActionStatus.COMPLETED)
                            rospy.loginfo('%s: We have arrived at the goal sir!',
                                          self._name)
                            rospy.loginfo('%s: \n Distance to goal\n x: %.2f\n y: %.2f\n z: %.2f\n',
                                          self._name,
                                          xref-x,
                                          yref-y,
                                          zref-z)




                    # reset the PID controllers
                    self.xPID.reset()
                    self.yPID.reset()
                    self.yawPID.reset()
                    cmd_vel = Twist()

                    ######################################
                    # send control signal                #
                    self._cmd_vel_pub.publish(cmd_vel)   #
                    ######################################


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
        self.set_action_status(ActionStatus.STARTED)
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
        rospy.loginfo('%s: Action_status: %d \n', self._name, self.get_action_status())
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
        
if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('bebop')
        # create the controller object
        c = BebopController(rospy.get_name())
        # start the controller
        c.run()
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"
