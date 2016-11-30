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


class PlannerInterface:
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


    def setup_problem(self):
        #updateSrv.request.update_type = KnowledgeUpdateService.ADD_KNOWLEDGE

        self.clear_knowledge()

        objects = {'waypoint': ['depo', 'wp0', 'wp1','wp2','wp3'],
                    'drone': ['drone0', 'drone1'],
                    'turtlebot': ['bot0', 'bot1'],
                    'person': ['person0', 'person1'],
                    'box': ['box0', 'box1', 'box2']}

        objects['airwaypoint'] = ["a_%s" % wp for wp in objects['waypoint']]

        at_dict = {'depo': objects['box'],
                   'wp0': ['person0'],
                   'wp1': ['person1'],
                   'a_wp2': ['drone0'],
                   'a_wp3': ['drone1']}

        # Even if the above lines will move to a config file, the following should hold as long as it is read as an object dictionary
        rospy.set_param('/available_drones', objects['drone'])
        rospy.set_param('/available_turtlebots', objects['turtlebot'])

        # objects = {'waypoint': ['depo', 'wp0'],
        #             'airwaypoint': ['a_depo', 'a_wp0'],
        #             'drone': ['drone0'],
        #             'turtlebot': [],
        #             'person': ['person0'],
        #             'box': ['box0']}

        # at_dict = {'depo': objects['box'],
        #            'wp0': ['person0'],
        #            'a_wp0': ['drone0']}

        empty_waypoints = []
        valid_occupants =  objects['drone'] + objects['turtlebot']
        for loc in objects['waypoint'] + objects['airwaypoint']:
            valid = False
            if loc in at_dict:
                for o in at_dict[loc]:
                    valid = True if (o in valid_occupants) else valid

            if not valid:
                empty_waypoints.append(loc)

       # Set up waypoint distances
        d = 20

        for wp1 in objects['waypoint']:
            for wp2 in objects['waypoint']:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FUNCTION
                kitem.attribute_name = "move-duration"
                kitem.values = [KeyValue('from', wp1), KeyValue('to', wp2)]
                kitem.function_value = 1 if wp1 == wp2 else d
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        for wp1 in objects['airwaypoint']:
            for wp2 in objects['airwaypoint']:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FUNCTION
                kitem.attribute_name = "move-duration"
                kitem.values = [KeyValue('from', wp1), KeyValue('to', wp2)]
                kitem.function_value = 1 if wp1 == wp2 else d
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Set empty for agents and waypoints
        for obj in objects['drone'] + objects['turtlebot'] + empty_waypoints:
            kitem = KnowledgeItem()
            kitem.knowledge_type = KnowledgeItem.FACT
            kitem.values = [KeyValue('aw_object', obj)]
            kitem.attribute_name = 'empty'
            self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Add agents
        for key, items in objects.iteritems():
            for item in items:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.INSTANCE
                kitem.instance_type = key
                kitem.instance_name = item
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Add locations for agents and boxes
        for key, values in at_dict.iteritems():
            for value in values:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FACT
                kitem.values = [KeyValue('object', value), KeyValue('waypoint', key)]
                kitem.attribute_name = 'at'
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Set relation between air and ground waypoints
        for air, ground in zip(objects['airwaypoint'], objects['waypoint']):
            kitem = KnowledgeItem()
            kitem.knowledge_type = KnowledgeItem.FACT
            kitem.values = [KeyValue('airwaypoint', air), KeyValue('waypoint', ground)]
            kitem.attribute_name = 'over'
            self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Set relation between air and ground waypoints
        for person in objects['person']:
            kitem = KnowledgeItem()
            kitem.knowledge_type = KnowledgeItem.FACT
            kitem.values = [KeyValue('person', person)]
            kitem.attribute_name = 'handled'
            self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_GOAL, knowledge=kitem)

    def start(self):
        self.cmd_pub.publish(StringMsg("plan"))

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    p_interface = plannerInterface()
    p_interface.setup_problem()
    p_interface.start()
    p_interface.spin()
