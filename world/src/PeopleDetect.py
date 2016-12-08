#!/usr/bin/env python
import rospy
import tf
from world.msg import TagRef
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

global pub,tags

def tagCallback(data):
  global pub,tags
  for detection in data.detections:
    if detection.id in tags:
      rospy.loginfo("Person {} detected at {}".format(detection.id, detection.pose))

def listener():
  global listener,pub,tags
  rospy.init_node('people_detector', anonymous=False)
  tags = []
  for s in rospy.get_param('~tags', "").split():
    tags.append(int(s))
  rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tagCallback) 
  rospy.spin()

if __name__ == "__main__":
  listener()

