#!/usr/bin/env python

import rospy

import csv
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch,ActionFeedback
from main import ActionName
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

        rospy.loginfo('action_feeder:__init__: Using actions from %s', wp_file)

        rospy.init_node('action_feeder', anonymous=True, log_level=rospy.INFO)

        self.action_pub = rospy.Publisher("/kcl_rosplan/action_dispatch", ActionDispatch, queue_size=10)

        rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, self.action_feedback_callback)
#        rospy.sleep(2)

    def read_actions(self, filename):
        actions = []
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) > 1:
                    actions.append([a.strip() for a in row])
            
        return actions


    def action_feedback_callback(self, msg):
        rospy.loginfo('action_feeder:Receiving feedback on action id: %i, status: %s', msg.action_id, msg.status)
        assert(self.current_msg.action_id == msg.action_id)
        if msg.status == "action_enabled":
            self.status = ActionStatus.RECEIVED
        elif msg.status == "action_failed":
            self.status = ActionStatus.FAILED
        elif msg.status == "action_achieved":
            self.status = ActionStatus.COMPLETED


    def dispatch(self):
        rospy.loginfo('action_feeder:Dispatching action id: %i', self.current_msg.action_id)
        r = rospy.Rate(10) # Hz

        self.action_pub.publish(self.current_msg)
        self.status = ActionStatus.SENT
        while (self.status < ActionStatus.FAILED):
            if rospy.is_shutdown():
                sys.exit()
            r.sleep()

        if (self.status == ActionStatus.FAILED):
            raise ActionFeedError("Action failed, aborting!")
    
    def run(self):
        while(self.action_pub.get_num_connections() < 1):
            rospy.sleep(0.1)
        for i, action in enumerate(self.actions):
            self.current_msg.action_id = i
            self.current_msg.name = action[0]
            self.current_msg.parameters = [KeyValue('obj',action[1])]
            if action[0] == ActionName.goto:
                self.current_msg.parameters.append(KeyValue('wp',action[2]))
            self.dispatch()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: action_feeder.py action_file")
    else:
        feeder = ActionFeeder(sys.argv[1])
        try:
            feeder.run()
        except rospy.ROSInterruptException:
            pass

