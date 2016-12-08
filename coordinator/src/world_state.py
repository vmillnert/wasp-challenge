#!/usr/bin/env python

import sys
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String as StringMsg
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from geometry_msgs.msg import Point
from coordinator.srv import *
from visualization_msgs.msg import MarkerArray, Marker

import rospy
import rospkg
import yaml
from copy import copy, deepcopy
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

        #Publisher for visualizing the state in rviz
        self.marker_pub = rospy.Publisher("~state_visualization", MarkerArray, queue_size=10, latch = True)
        self.vis_markers = {}

        # Variables for keeping track of the world
        self.objects = {}
        self.at = {}
        self.waypoint_positions = {}
        self.carrying = {}
        self.rescued = []

        # Reverse lookup for convenience, ONLY FOR READING
        self.obj2loc = {}

        # Read config file
        self.read_world_config()

        # Setup own services for coordinator
        self.waypoint_pos_service = rospy.Service('~get_waypoint_position', WaypointPosition, self.get_waypoint_position)
        self.replan_service = rospy.Service('~plan', Empty, self.start_planner)
        self.action_finished_service = rospy.Service('~action_finished', ActionFinished, self.action_finished_cb)
        self.read_config_service = rospy.Service('~read_world_config', Empty, self.read_world_config_cb)


    def read_world_config_cb(self, request):
        self.read_world_config()
        return EmptyResponse()

    def read_world_config(self):
        if self.world_config_file == None:
            # No config file, start with empty world
            return False

        #Load yaml file
        with open(self.world_config_file) as f:
            yaml_world = yaml.load(f)
        self.objects = deepcopy(yaml_world['objects'])
        self.at = deepcopy(yaml_world['at'])
        self.waypoint_positions = deepcopy(yaml_world['waypoints'])

        # Add corresponding air waypoints
        self.objects['airwaypoint'] = []
        for wp in self.objects['waypoint']:
            air_wp = "a_%s" % wp
            self.objects['airwaypoint'].append(air_wp)
            self.waypoint_positions[air_wp] = self.waypoint_positions[wp]

        # Initiate carrying dict
        if 'turtlebot' in self.objects:
            self.carrying = {agent: [] for agent in (self.objects['drone'] + self.objects['turtlebot'])}
        else:
            self.carrying = {agent: [] for agent in (self.objects['drone'])}


        # Initate empty waypoints
        for wp in (self.objects['waypoint'] + self.objects['airwaypoint']):
            if not (wp in self.at):
                self.at[wp] = []

        rospy.loginfo('/%s/read_world_config/World config file %s, has been read.' % (self.node_name, self.world_config_file))

        # TODO: Move to topics instead. Use latch and only publish a new topic when there is a change
        rospy.set_param('/available_drones', self.objects['drone'])
        if 'turtlebot' in self.objects:
            rospy.set_param('/available_turtlebots', self.objects['turtlebot'])
        else:
            rospy.set_param('/available_turtlebots', [])

        #  Generate reverse lookup for objects and locations
        self._generate_obj2loc()
        self.visualize_state()

        return True

    def generate_knowledge_base(self):

        # Remove earlier entries from knowledge database
        self.clear_knowledge()

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
        # No world update when using the primitive action feeder
        if not self.use_rosplan:
          return ActionFinishedResponse()

        action = action_req.action
        params = {p.key: p.value for p in action.parameters}

        #Early exit for unsuccessful action
        #TODO: Special handling of some action. E.g. Add new waypoint if goto or fly cancelled
        if not action_req.success:
            pass

        elif action.name == 'goto' or action.name == 'fly':
            for loc, objs in self.at.iteritems():
                if params['agent'] in objs:
                    objs.remove(params['agent'])
            self.at[params['to']].append(params['agent'])

        elif action.name == 'pick-up':
            self.at[params['ground']].remove(params['box'])
            self.carrying[params['drone']].append(params['box'])

        elif action.name == 'hand-over-drone2bot':
            self.carrying[params['turtlebot']].append(params['box'])
            self.carrying[params['drone']].remove(params['box'])

        elif action.name == 'hand-over-bot2drone':
            self.carrying[params['drone']].append(params['box'])
            self.carrying[params['turtlebot']].remove(params['box'])

        elif action.name == 'unload':
            self.carrying[params['drone']].remove(params['box'])
            self.objects['box'].remove(params['box'])
            self.rescued.append(params['person'])

        elif action.name == 'takeoff':
            for loc, objs in self.at.iteritems():
                if params['drone'] in objs:
                    objs.remove(params['drone'])
            self.at[params['air']].append(params['drone'])

        elif action.name == 'land':
            for loc, objs in self.at.iteritems():
                if params['drone'] in objs:
                    objs.remove(params['drone'])
            self.at[params['ground']].append(params['drone'])

        # Generate reverse lookup for objects and locations
        self._generate_obj2loc()
        self.visualize_state()

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
        print "Carrying:"
        for wp, objs in self.carrying.iteritems():
            print "%s: %s" % (wp, ",".join(objs))

        print "--------------------------------"

    def visualize_state(self):
        # List of active objects, used to remove old ones
        active_objs = set()
        #Go through locations
        for obj_type, objs in self.objects.iteritems():
            # Skip waypoints
            if obj_type in ['airwaypoint', 'waypoint']:
                continue

            for obj in objs:
                active_objs.add(obj)
                #Add object if it does not exist
                if obj not in self.vis_markers:
                    self._create_visualization_marker(obj, obj_type)

                #Update object
                self._update_visualization_marker(obj, obj_type)

        #Check for old objects, need to notify rviz to remove them
        old_objects = set(self.vis_markers).difference(active_objs)
        for old_obj in old_objects:
            self._remove_visualization_marker(old_obj)

        #Create array message
        marray = MarkerArray()
        marray.markers = []
        for obj, markers in self.vis_markers.iteritems():
            marray.markers += markers

        # Publish!
        self.marker_pub.publish(marray)

        # Remove old objects from dict
        for old_obj in old_objects:
            del self.vis_markers[old_obj]

    def _create_visualization_marker(self, obj, obj_type):
        # Text marker to display object name
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time() # Time zero, always show
        text_marker.ns = obj
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.text = obj
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.scale.x = 0.2
        text_marker.scale.y = 0.2
        text_marker.scale.z = 0.1

        #Set position, depends on type
        if obj_type  in ["drone", "turtlebot"]:
            text_marker.header.frame_id = "/%s/base_link" % obj if obj_type == "drone" else "/base_link"
            text_marker.frame_locked = True
            text_marker.pose.position.x = 0.0
            text_marker.pose.position.y = 0.0
            text_marker.pose.position.z = 0.1
        elif obj_type == "person":
            wp_pos = self.waypoint_positions[self.obj2loc[obj]]
            text_marker.pose.position.x = wp_pos['x']
            text_marker.pose.position.y = wp_pos['y']
            text_marker.pose.position.z = 0.1

        #Early exit for types that only needs text
        if obj_type == "turtlebot":
            self.vis_markers[obj] = [text_marker]
            return

        # Colored marker
        marker = deepcopy(text_marker)
        marker.id = 1
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        marker.color.a = 0.5

        if obj_type  == "drone":
            marker.type = Marker.SPHERE
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif obj_type == "box":
            marker.type = Marker.CUBE
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif obj_type == "person":
            marker.type = Marker.CYLINDER
            #Set color in update step

        self.vis_markers[obj] = [text_marker,marker]

    def _update_visualization_marker(self, obj, obj_type):
        for marker in self.vis_markers[obj]:
            #Set position for box, can move during execution
            if obj_type == "box":
                # Box or person at waypoint
                if self.obj2loc[obj] in self.objects['waypoint']:
                    wp_pos = self.waypoint_positions[self.obj2loc[obj]]
                    marker.pose.position.x = wp_pos['x']
                    marker.pose.position.y = wp_pos['y']
                    try:
                        marker.pose.position.z = float(obj[-1]) * marker.scale.z
                    except ValueError:
                        marker.pose.position.z = 0.0

                #Box with agent
                else:
                    marker.header.frame_id = "/%s/base_link" % obj
                    marker.frame_locked = True
                    marker.pose.position.x = 0.0
                    marker.pose.position.y = 0.0
                    marker.pose.position.z = 0.5
            #Only change geometric shape color on person
            elif obj_type == "person" and marker.id == 1:
                if obj in self.rescued:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0

    def _remove_visualization_marker(self, obj):
        for marker in self.vis_markers[obj]:
            marker.action = Marker.DELETE

    def _generate_obj2loc(self):
        self.obj2loc = {}
        for wp, objs in self.at.iteritems():
            for obj in objs:
                self.obj2loc[obj] = wp
        for agent, objs in self.carrying.iteritems():
            for obj in objs:
                self.obj2loc[obj] = agent

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    ws = WorldState()
    ws.spin()
