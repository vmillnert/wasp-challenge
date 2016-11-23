#!/usr/bin/env python

import sys
from std_msgs.msg import String as StringMsg
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService

import rospy

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
        

        self.setup_problem()


    def setup_problem(self):
            #updateSrv.request.update_type = KnowledgeUpdateService.ADD_KNOWLEDGE
            wps = ['depo', 'wp0', 'wp1']
            for wp in wps:
                kitem = KnowledgeItem()
                kitem.knowledge_type = KnowledgeItem.INSTANCE
                kitem.instance_type = "waypoint"
                kitem.instance_name = wp
                self.update_knowledge(update_type=1, knowledge = kitem)


    def start(self):
        self.cmd_pub.publish(StringMsg("plan"))

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    p_interface = plannerInterface()
    p_interface.start()
    p_interface.spin()
