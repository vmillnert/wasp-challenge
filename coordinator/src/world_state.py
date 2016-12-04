#!/usr/bin/env python

import sys
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String as StringMsg
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from geometry_msgs.msg import Point
from coordinator.srv import *

import rospy
import rospkg
import yaml
from copy import copy
import os
from math import sqrt


class KnowledgeUpdateServiceEnum:
    ADD_KNOWLEDGE = 0
    ADD_GOAL = 1
    REMOVE_KNOWLEDGE = 2
    REMOVE_GOAL = 3


class WorldState:
    def __init__(self):
        
        # Initiate node
        self.node_name = "world_state"
        rospy.init_node(self.node_name, anonymous=False, log_level=rospy.INFO)

        # Initiate planner
        rospy.loginfo('/%s/__init__/' % self.node_name)

        # Get parameters
        self.use_rosplan = rospy.get_param('~use_rosplan', True)
        rospack = rospkg.RosPack()
        self.world_config_file = rospy.get_param("~world_config_file",
                                                 os.path.join(rospack.get_path('coordinator'), 'config','test_world.yaml'))

        # Set up Publisher
        self.cmd_pub = rospy.Publisher("/kcl_rosplan/planning_commands", StringMsg, queue_size=10, latch = True)

        # Set up knowledge update service
        knowledge_service_name = "/kcl_rosplan/update_knowledge_base"
        if self.use_rosplan:
            rospy.wait_for_service(knowledge_service_name)
        self.update_knowledge = rospy.ServiceProxy(knowledge_service_name, KnowledgeUpdateService)

        knowledge_service_name = "/kcl_rosplan/clear_knowledge_base"
        if self.use_rosplan:
            rospy.wait_for_service(knowledge_service_name)
        self.clear_knowledge = rospy.ServiceProxy(knowledge_service_name, Empty)

        # Setup own services for coordinator
        self.waypoint_pos_service = rospy.Service('~get_waypoint_position', WaypointPosition, self.get_waypoint_position)
        self.replan_service = rospy.Service('~plan', Empty, self.start_planner)
        self.action_finished_service = rospy.Service('~action_finished', ActionFinished, self.action_finished_cb)

        # Variables for keeping track of the world
        self.objects = {}
        self.at = {}
        self.waypoint_positions = {}
        self.carrying = {}
        self.rescued = []

        # Read config file
        self.read_world_config()

    
    def read_world_config(self):
        if self.world_config_file == None:
            # No config file, start with empty world
            return False

        #Load yaml file
        with open(self.world_config_file) as f:
            yaml_world = yaml.load(f)
        self.objects = yaml_world['objects']
        self.at = yaml_world['at']
        self.waypoint_positions = yaml_world['waypoints']

        # Add corresponding air waypoints
        self.objects['airwaypoint'] = []
        for wp in self.objects['waypoint']:
            air_wp = "a_%s" % wp
            self.objects['airwaypoint'].append(air_wp)
            self.waypoint_positions[air_wp] = self.waypoint_positions[wp]

        # Initiate carrying dict
        self.carrying = {agent: [] for agent in (self.objects['drone'] + self.objects['turtlebot'])}
        
        # Initate empty waypoints
        for wp in (self.objects['waypoint'] + self.objects['airwaypoint']):
            if not (wp in self.at):
                self.at[wp] = []

        # TODO: Move to topics instead. Use latch and only publish a new topic when there is a change
        rospy.set_param('/available_drones', self.objects['drone'])
        rospy.set_param('/available_turtlebots', self.objects['turtlebot'])

        return True

    def generate_knowledge_base(self):

        # Remove earlier entries from knowledge database
        self.clear_knowledge()

        # Build a new at dict where drones are in the corresponding air waypoint
        # TODO: Remove this when we have takeoff and land actions
        new_at = copy(self.at)
        for wp, objs in self.at.iteritems():
            new_objs = copy(objs)
            
            # Only look for drones at ground waypoints
            if wp not in self.objects['waypoint']:
                continue
            # Remove drone from current waypoint, create new air waypoint
            for drone in self.objects['drone']:
                if drone in new_objs:
                    new_at[wp].remove(drone)
                    new_at["a_%s" % wp].append(drone)

        self.at = new_at

        empty_waypoints = []
        valid_occupants =  self.objects['drone'] + self.objects['turtlebot']
        for loc in self.objects['waypoint'] + self.objects['airwaypoint']:
            valid = False
            if loc in self.objects:
                for o in self.objects[loc]:
                    valid = True if (o in valid_occupants) else valid

            if not valid:
                empty_waypoints.append(loc)

       # Set up waypoint distances
        for wp1 in self.objects['waypoint']:
            for wp2 in self.objects['waypoint']:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FUNCTION
                kitem.attribute_name = "move-duration"
                kitem.values = [KeyValue('from', wp1), KeyValue('to', wp2)]
                kitem.function_value = self.waypoint_distance(wp1,wp2)
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        for wp1 in self.objects['airwaypoint']:
            for wp2 in self.objects['airwaypoint']:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FUNCTION
                kitem.attribute_name = "move-duration"
                kitem.values = [KeyValue('from', wp1), KeyValue('to', wp2)]
                kitem.function_value = self.waypoint_distance(wp1,wp2)
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Set empty for agents and waypoints
        for obj in self.objects['drone'] + self.objects['turtlebot'] + empty_waypoints:
            kitem = KnowledgeItem()
            kitem.knowledge_type = KnowledgeItem.FACT
            kitem.values = [KeyValue('aw_object', obj)]
            kitem.attribute_name = 'empty'
            self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Add agents
        for key, items in self.objects.iteritems():
            for item in items:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.INSTANCE
                kitem.instance_type = key
                kitem.instance_name = item
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Add locations for agents and boxes
        for key, values in self.at.iteritems():
            for value in values:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.FACT
                kitem.values = [KeyValue('object', value), KeyValue('waypoint', key)]
                kitem.attribute_name = 'at'
                self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Set relation between air and ground waypoints
        for air, ground in zip(self.objects['airwaypoint'], self.objects['waypoint']):
            kitem = KnowledgeItem()
            kitem.knowledge_type = KnowledgeItem.FACT
            kitem.values = [KeyValue('airwaypoint', air), KeyValue('waypoint', ground)]
            kitem.attribute_name = 'over'
            self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_KNOWLEDGE, knowledge=kitem)

        # Set goals, each person should be handled
        for person in self.objects['person']:
            kitem = KnowledgeItem()
            kitem.knowledge_type = KnowledgeItem.FACT
            kitem.values = [KeyValue('person', person)]
            kitem.attribute_name = 'handled'
            self.update_knowledge(update_type=KnowledgeUpdateServiceEnum.ADD_GOAL, knowledge=kitem)

    def waypoint_distance(self, wp1, wp2):
        if wp1 == wp2:
            return 0

        p1 = self.waypoint_positions[wp1]
        p2 = self.waypoint_positions[wp2]
        d = sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)
        
        return d

    def get_waypoint_position(self,wp_req):
        try:
            wp_position = self.waypoint_positions[wp_req.wp]
            p_srv = WaypointPositionResponse(x = wp_position['x'], y = wp_position['y'], valid = True)
        except KeyError:
            p_srv = WaypointPositionResponse(x = 0, y = 0, valid = False)

        return p_srv

    def action_finished_cb(self,action_req):
        action = action_req.action
        params = {p.key: p.value for p in action.parameters}

        if action.name == 'goto':
#            self.at[params['from']].remove(params['agent'])
            self.at[params['to']].append(params['agent'])

        elif action.name == 'pick-up':
            self.at[params['ground']].remove(params['box'])
            self.carrying[params['drone']].append(params['box'])

        elif action.name == 'hand-over':
            self.carrying[params['turtlebot']].append(params['box'])
            self.carrying[params['drone']].remove(params['box'])

        elif action.name == 'unload':
            self.carrying[params['drone']].remove(params['box'])
            self.rescued.append(params['person'])

        return ActionFinishedResponse()

    def start_planner(self, request):
        rospy.loginfo('/%s/start_planner/ Generating knowledge base and starting planner' % self.node_name)
        self.generate_knowledge_base()
        self.cmd_pub.publish(StringMsg("plan"))

        return EmptyResponse()

    def print_state(self):
        print "-----------World State----------"
        print "Persons rescued: %s" % ",".join(self.rescued)
        print "Locations:"
        for wp, objs in self.at.iteritems():
            print "%s: %s" % (wp, ",".join(objs))
        print "--------------------------------"

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    ws = WorldState()
    ws.spin()
