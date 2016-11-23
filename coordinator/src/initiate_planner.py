#!/usr/bin/env python

import sys
from std_srvs.srv import Empty
from std_msgs.msg import String as StringMsg
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService

import rospy


class KnowledgeUpdateServiceEnum:
    ADD_KNOWLEDGE = 0
    ADD_GOAL = 1
    REMOVE_KNOWLEDGE = 2
    REMOVE_GOAL = 3


class plannerInterface:
    def __init__(self):
        # Initiate node
        rospy.init_node('planner_interface', anonymous=True, log_level=rospy.INFO)

        # Initiate planner
        rospy.loginfo('/plannerInterface/__init__/')

        # Set up Publisher
        self.cmd_pub = rospy.Publisher("/kcl_rosplan/planning_commands", StringMsg, queue_size=10, latch = True)

        # Set up knowledge update service
        knowledge_service_name = "/kcl_rosplan/update_knowledge_base"
        rospy.wait_for_service(knowledge_service_name)
        self.update_knowledge = rospy.ServiceProxy(knowledge_service_name, KnowledgeUpdateService)

        rospy.wait_for_service("/kcl_rosplan/clear_knowledge_base")
        self.clear_knowledge = rospy.ServiceProxy("/kcl_rosplan/clear_knowledge_base", Empty)

        self.setup_problem()


    def setup_problem(self):
        #updateSrv.request.update_type = KnowledgeUpdateService.ADD_KNOWLEDGE

        self.clear_knowledge()

        objects = {'LOC': ['depo', 'wp0', 'wp1'],
                    'DRONE': ['drone0'],
                    'PERSON': ['person0', 'person1'],
                    'BOX': ['box0', 'box1', 'box2'],
                    'BTYPE': ['med', 'food', 'water']}

        at_dict = {'depo': objects['BOX'] + objects['DRONE'],
                   'wp0': ['person0'],
                   'wp1': ['person1']}

        goal_dict = {'person0': ['med', 'food'],
                     'person1': ['med']}

        box_dict = {'med': ['box0', 'box1'],
                    'food': ['box2']}
        d = 20
        d_matrix = [[0, d, d],[d, 0, d],[d, d, 0]]

        for i, wp1 in enumerate(objects['LOC']):
            for j, wp2 in enumerate(objects['LOC']):

                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FUNCTION
                kitem.attribute_name = "fly-cost"
                kitem.values = [KeyValue('from', wp1), KeyValue('to', wp2)]
                kitem.function_value = d_matrix[i][j]
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        for drone in objects['DRONE']:
            kitem = KnowledgeItem()
            kitem.knowledge_type = KnowledgeItem.FACT
            kitem.values = [KeyValue('d', drone)]
            kitem.attribute_name = 'empty'
            self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        for key, items in objects.iteritems():
            for item in items:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.INSTANCE
                kitem.instance_type = key
                kitem.instance_name = item
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        for key, values in at_dict.iteritems():
            for value in values:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FACT
                kitem.values = [KeyValue('o', value), KeyValue('l', key)]
                kitem.attribute_name = 'at'
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        for btype, boxes in box_dict.iteritems():
            for box in boxes:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FACT
                kitem.values = [KeyValue('b', box), KeyValue('t', btype)]
                kitem.attribute_name = 'contains'
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        for key, items in goal_dict.iteritems():
            for item in items:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FACT
                kitem.values = [KeyValue('p', key), KeyValue('t', item)]
                kitem.attribute_name = 'has'
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_GOAL, knowledge=kitem)


    def start(self):
        self.cmd_pub.publish(StringMsg("plan"))

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    p_interface = plannerInterface()
    p_interface.start()
    p_interface.spin()
