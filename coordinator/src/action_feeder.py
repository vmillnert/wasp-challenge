#!/usr/bin/env python

import rospy

import csv
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch,ActionFeedback
from main import Action
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


class ActionFeeder:
    def __init__(self, wp_file):
        self.actions = self.read_actions(wp_file)
        self.current_msg = ActionDispatch()
        self.status = ActionStatus.START

        rospy.logdebug('action_feeder:__init__: Using actions from %s', wp_file)

        rospy.init_node('action_feeder', anonymous=True, log_level=rospy.DEBUG)

        rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, self.action_feedback_callback)

        self.action_pub = rospy.Publisher("/kcl_rosplan/action_dispatch", ActionDispatch, queue_size=10)
        rospy.sleep(0.1)


    def read_actions(self, filename):
        actions = None
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            actions = [r for r in reader]
            
        return actions


    def action_feedback_callback(self, msg):
        rospy.logdebug('action_feeder:action_dispatch_callback:%s', msg.name)
        assert(self.current_msg.action_id == msg.action_id)
        if msg.status == "action_enabled":
            self.status = ActionStatus.RECEIVED
        elif msg.status == "action_failed":
            self.status = ActionStatus.FAILED
        elif msg.status == "action_achieved":
            self.status = ActionStatus.COMPLETED


    def dispatch(self):
        r = rospy.Rate(0.1)

        self.action_pub.publish(self.current_msg)
        self.status = ActionStatus.SENT
        try:
            while self.status < ActionStatus.FAILED:
                r.sleep()
        except KeyboardInterrupt:
            rospy.loginfo('action_feeder:interrupted:')
            sys.exit(0)

        if (status == ActionStatus.FAILED):
            raise ActionFeedError("Action failed, aborting!")
    
    def run(self):
        for i, action in enumerate(self.actions):
            self.current_msg.action_id = i
            self.current_msg.name = action[0]
            self.current_msg.parameters = [KeyValue('obj',action[1])]
            if action[0] == Action.goto:
                self.current_msg.parameters.append(KeyValue('wp',action[2]))
            self.dispatch()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: action_feeder.py action_file")
    else:
        feeder = ActionFeeder(sys.argv[1])
        feeder.run()
