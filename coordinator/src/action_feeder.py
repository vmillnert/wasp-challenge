#!/usr/bin/env python

import rospy

import csv
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch,ActionFeedback
from main import ActionName
from threading import Lock
import sys

class ActionStatus:
    START = 0
    SENT = 1
    RECEIVED = 2
    FAILED = 3
    COMPLETED = 4


class ActionFeedError(Exception):
     def __init__(self, value):
         self.value = value
     def __str__(self):
         return repr(self.value)

class Action:
  def __init__(self, _id, command):
    self._id = _id
    self.command = command
    self.status = ActionStatus.START
 

class ActionFeeder:
    def __init__(self, wp_file):
        self.actions = self.read_actions(wp_file)
        self.status = ActionStatus.START
        self.lock = Lock()
        self.pending = []

        rospy.loginfo('action_feeder:__init__: Using actions from %s', wp_file)

        rospy.init_node('action_feeder', anonymous=True, log_level=rospy.INFO)

        self.action_pub = rospy.Publisher("/kcl_rosplan/action_dispatch", ActionDispatch, queue_size=10)

        rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, self.action_feedback_callback)
        rospy.sleep(2)

    def read_actions(self, filename):
        actions = []
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            i = 0
            for row in reader:
                if not len(row) == 0:
                    actions.append(Action(i, [a.strip() for a in row]))
                i += 1
        return actions


    def action_feedback_callback(self, msg):
        rospy.loginfo('action_feeder:Receiving feedback on action id: %i, status: %s', msg.action_id, msg.status)
        self.lock.acquire()
        assert(msg.action_id in self.pending)
        assert(self.actions[msg.action_id].status <= ActionStatus.RECEIVED)
        if msg.status == "action enabled":
            self.actions[msg.action_id].status = ActionStatus.RECEIVED
        else:
          if msg.status == "action failed":
            self.actions[msg.action_id].status = ActionStatus.FAILED
          elif msg.status == "action achieved":
            self.actions[msg.action_id].status = ActionStatus.COMPLETED
          self.pending.remove(msg.action_id)
        self.lock.release()


    def run(self):
        while(self.action_pub.get_num_connections() < 1):
            rospy.sleep(0.1)
        r = rospy.Rate(10) # Hz
        for i, action in enumerate(self.actions):
            if action.command[0] == 'wait':
              rospy.loginfo('action_feeder:Waiting %i', msg.action_id)
              self.lock.acquire()
              while not rospy.is_shutdown() and len(self.pending) > 0:
                self.lock.release()
                r.sleep()
                self.lock.acquire()
              self.lock.release()
              action.status = ActionStatus.COMPLETED
            else:
              msg = ActionDispatch()
              msg.action_id = action._id
              msg.name = action.command[0]
              msg.parameters = [KeyValue('agent',action.command[1])]
              if action.command[0] == ActionName.goto:
                msg.parameters.append(KeyValue('to',action.command[2]))

              rospy.loginfo('action_feeder:Dispatching action id: %i', msg.action_id)
              action.status = ActionStatus.SENT
              self.lock.acquire()
              self.pending.append(action._id)
              self.lock.release()
              self.action_pub.publish(msg)

            if rospy.is_shutdown():
              sys.exit()


if __name__ == '__main__':

    print('Trying to initialize the action_feeder')

    if len(sys.argv) < 2:
        print("usage: action_feeder.py action_file")
    else:
        feeder = ActionFeeder(sys.argv[1])
        try:
            feeder.run()
        except rospy.ROSInterruptException:
            pass

