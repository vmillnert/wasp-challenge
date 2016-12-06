#!/usr/bin/env python

import sys
import csv

import rospy
from actionlib import SimpleActionClient

from actionlib_msgs.msg import GoalStatus
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from bebop_controller.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from coordinator.srv import *
from threading import RLock


class ActionName:
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
        self.msg = action_dispatch_msg
        self.action_id = action_dispatch_msg.action_id
        self.name = action_dispatch_msg.name

        self.parameters = action_dispatch_msg.parameters

        self.agent = None
        self.wp = None

        for p in self.parameters:
            if p.key == "agent" or p.key == "drone":
                self.agent = p.value
            if p.key == "to":
                self.wp = p.value

        if self.agent == None:
            raise CoordinatorError("No key agent in action dispatch parameter")
        if self.wp == None and self.name == ActionName.goto:
            raise CoordinatorError("No key wp in action dispatch parameter")

class Client:
  def __init__(self, name):
    self.name = name          # Name of the agent
    self.actions = {}         # Action client instances
    self.action = False       # Ongoing action
    self.ac = False           # Action client for the ongoing action
    self.queue = []           # Queued actions
    self._lock = RLock()      # Thread lock

  def wait_for(self):
    for k in self.actions.keys():
      self.actions[k].wait_for_server()

  def is_idle(self):
    return self.action == False

  def set_action(self, action):
    self.action = action

  def queue_action(self, action):
    self.queue.append(action)

  def lock(self):
    self._lock.acquire()

  def unlock(self):
    self._lock.release()

  def read_queue(self):
    if len(self.queue) == 0:
      return False
    return self.queue.pop(0)


class TurtleClient(Client):
  def __init__(self, name):
    Client.__init__(self, name)
    self.actions = { 'goto': SimpleActionClient("move_base", MoveBaseAction) }

class DroneClient(Client):
  def __init__(self, name):
    Client.__init__(self, name)
    self.actions = {
        'land' : SimpleActionClient(name+"/BebopLandAction", BebopLandAction),
        'load' : SimpleActionClient(name+"/BebopLoadAction", BebopLoadAction),
        'goto' : SimpleActionClient(name+"/BebopMoveBaseAction", BebopMoveBaseAction),
        'takeoff' : SimpleActionClient(name+"/BebopTakeOffAction", BebopTakeOffAction),
        'unload' : SimpleActionClient(name+"/BebopUnloadAction", BebopUnloadAction),
        'follow' : SimpleActionClient(name+"/BebopFollowAction", BebopFollowAction),
    }

class Coordinator:
    def __init__(self, wp_file):
        self.clients = {}

        rospy.init_node('coordinator', anonymous=True, log_level=rospy.INFO)

        rospy.loginfo('/coordinator/__init__/ - Using waypoints from %s', wp_file)

        # Set up Publisher
        self.feedback_pub = rospy.Publisher("/kcl_rosplan/action_feedback", ActionFeedback, queue_size=10)

        # Wait for WorldState - it populates the list of agents and must be available before accepting any actions
        rospy.loginfo("* Coordinator is waiting for world state update handler")
        waypoint_srv_name = 'world_state/get_waypoint_position'
        rospy.wait_for_service(waypoint_srv_name)
        self.get_waypoint_position = rospy.ServiceProxy(waypoint_srv_name, WaypointPosition)
        srv_name = 'world_state/action_finished'
        rospy.wait_for_service(srv_name)
        self.action_finished_update = rospy.ServiceProxy(srv_name, ActionFinished)

        for agent in rospy.get_param('/available_drones'):
          print("Adding drone: {}".format(agent))
          self.clients[agent] = DroneClient(agent)
        for agent in rospy.get_param('/available_turtlebots'):
          print("Adding turtlebot: {}".format(agent))
          self.clients[agent] = TurtleClient(agent)

        rospy.loginfo('* Coordinator is waiting for action clients')
        for k in self.clients.keys():
          rospy.loginfo("* Wait for client {}".format(k))
          self.clients[k].wait_for()
        rospy.loginfo('* All action clients have started')

        self.coordinate_frame = rospy.get_param("~coordinate_frame", "map")

        # Interface to ROSplan
        rospy.Subscriber("/kcl_rosplan/action_dispatch", ActionDispatch, self.action_dispatch_callback)


    def action_dispatch_callback(self, msg):
        # Check Action type and call correct functions.
        action = Action(msg)
        client = self.clients[action.agent]
        client.lock()
        rospy.loginfo('\033[92m/coordinator/action_dispatch_callback agent "%s", action "%s" \033[0m', action.agent, action.name)

        if not client.is_idle():
          rospy.loginfo('Agent "%s" is occupied, action queued', action.agent)
          client.queue_action(msg)
        else:
          client.set_action(action)
          # Shared actions between action feeder and planner
          if (msg.name == 'goto' or msg.name == 'fly'):
            client.ac = self.action_goto(action)
          elif (msg.name == 'takeoff'):
            client.ac = self.action_takeoff(action)
          elif (msg.name == 'land'):
            client.ac = self.action_land(action)
          elif (msg.name == 'load'):
            client.ac = self.action_load(action)
          elif (msg.name == 'unload'):
            client.ac = self.action_unload(action)
          elif (msg.name == 'follow'):
            client.ac = self.action_follow(action)
          #Actions only defined by planner
          elif (msg.name == 'pick-up'):
            client.ac = self.action_load(action)
          elif (msg.name == 'hand-over-drone2bot'):
            client.ac = self.action_unload(action)
          elif (msg.name == 'hand-over-bot2drone'): 
            client.ac = self.action_load(action)
          else:
            rospy.loginfo("No action called %s for agent %s", msg.name, msg.agent)
            client.set_action(False)
        client.unlock()

    def _action_feedback_from_state(self, action, state):
        success = (state == GoalStatus.SUCCEEDED)

        # Feedback to rosplan
        feedback_msg = ActionFeedback()
        feedback_msg.action_id = action.action_id
        feedback_msg.status = "action achieved" if success else "action failed"
        self.feedback_pub.publish(feedback_msg)

        # Update world state
        update_request = ActionFinishedRequest(action = action.msg, success = success)
        self.action_finished_update(update_request)


    def action_takeoff(self, action):
        rospy.loginfo('/coordinator/action_takeoff for %s', action.agent)
        action_id = action.action_id
        feedback_msg = ActionFeedback(action_id=action_id, status="action enabled")
        ac = self.clients[action.agent].actions['takeoff']
        ac.send_goal(BebopTakeOffGoal())
        return ac


    def action_land(self, action):
        rospy.loginfo('/coordinator/action_land for %s', action.agent)
        action_id = action.action_id
        feedback_msg = ActionFeedback(action_id=action_id, status="action enabled")
        ac = self.clients[action.agent].actions['land']
        ac.send_goal(BebopLandGoal())
        return ac


    def action_load(self, action):
        rospy.loginfo('/coordinator/action_load for %s', action.agent)
        action_id = action.action_id
        feedback_msg = ActionFeedback(action_id=action_id, status="action enabled")
        ac = self.clients[action.agent].actions['load']
        ac.send_goal(BebopLoadGoal())
        return ac


    def action_unload(self, action):
        rospy.loginfo('/coordinator/action_unload for %s', action.agent)
        action_id = action.action_id
        feedback_msg = ActionFeedback(action_id=action_id, status="action enabled")
        ac = self.clients[action.agent].actions['unload']
        ac.send_goal(BebopUnloadGoal())
        return ac


    def action_follow(self, action):
        rospy.loginfo('/coordinator/action_follow for %s', action.agent)
        action_id = action.action_id
        feedback_msg = ActionFeedback(action_id=action_id, status="action enabled")
        ac = self.clients[action.agent].actions['follow']
        ac.send_goal(BebopFollowGoal())
        return ac


    def action_goto(self, action):
    	rospy.loginfo('/coordinator/action_goto for %s', action.agent)
        parameters = action.parameters
        action_id = action.action_id
        agent = action.agent
        wp = action.wp

        ac = None
        if agent in rospy.get_param('/available_drones'):
            goal = BebopMoveBaseGoal()
            ac = self.clients[action.agent].actions['goto']
        elif agent in rospy.get_param('/available_turtlebots'):
            goal = MoveBaseGoal()
            ac = self.clients[agent].actions['goto']
        if ac == None:
            raise CoordinatorError("No action client exist for agent %s" % agent)

        if not ac.wait_for_server(timeout=rospy.Duration(10)):
            rospy.loginfo("server timeout")

        #Get waypoint position from world state node
        p_rsp = self.get_waypoint_position(wp = wp)
        if not p_rsp.valid:
            raise CoordinatorError("No valid position for waypoint %s" % wp)

        goal.target_pose.header.frame_id = self.coordinate_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p_rsp.x
        goal.target_pose.pose.position.y = p_rsp.y
        goal.target_pose.pose.orientation.w = 1

        # Notify action dispatcher of status
        feedback_msg = ActionFeedback(action_id = action_id, status = "action enabled")
        self.feedback_pub.publish(feedback_msg)

        ac.send_goal(goal)
        return ac


    def test_actions(self):
        dispatch_msg = ActionDispatch()
        dispatch_msg.name = "goto"
        dispatch_msg.parameters = [KeyValue('agent','turtle'), KeyValue('wp','wp1')]
        tmp_pub = rospy.Publisher("/kcl_rosplan/action_dispatch", ActionDispatch, queue_size=1)
        rospy.sleep(0.1)
        tmp_pub.publish(dispatch_msg)
        rospy.logdebug('coordinator:test_actions')


    def spin(self):
      r = rospy.Rate(10) # Hz
      while not rospy.is_shutdown():
        for c in self.clients.values():
          c.lock()
          if not c.is_idle():
            s = c.ac.get_state()
            if not s in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
              self._action_feedback_from_state(c.action, s)
              c.set_action(False)
              msg = c.read_queue()
              if msg:
                self.action_dispatch_callback(msg)
          c.unlock()
        r.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: coordinator.py waypoint_file")
    else:
        coordinator = Coordinator(sys.argv[1])
        coordinator.spin()
