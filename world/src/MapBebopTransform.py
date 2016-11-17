#!/usr/bin/env python
import rospy
import tf
from world.msg import TagRef
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

global pub

def tagCallback(data):
  global pub
  for detection in data.detections:
    if detection.id == 0:
      tagref = TagRef()
      tagref.global_frame = "map"
      tagref.tag_frame = "plate_top_link"
      tagref.target_frame = "bebop/odom"
      tagref.pose = detection.pose
      pub.publish(tagref)

def listener():
  global listener, pub
  rospy.init_node('bebop_tagref', anonymous=False)
  pub = rospy.Publisher('tagref', TagRef, queue_size=10)
  rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tagCallback) # TODO: Name and type of topic
  rospy.spin()

if __name__ == "__main__":
  listener()

