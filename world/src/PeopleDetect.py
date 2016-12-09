#!/usr/bin/env python
import rospy
import tf
from world.msg import TagRef
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from coordinator.srv import AddObject
import numpy

global pub,tags

def tagCallback(data):
  global pub,tags
  for detection in data.detections:
    if detection.id in tags:
      rospy.loginfo("Person {} detected at {}".format(detection.id, detection.pose))
      add_person(detection)

def add_person(detection):
  global tag_pos,req_observations,add_object_service,map_frame
  # Init tracking
  if not detection.id in tag_pos:
    tag_pos[detection.id] = [numpy.zeros(2), 0]

  #Get position in map frame
  listener.waitForTransform(map_frame, detection.pose.header.frame_id,
                            detection.pose.header.stamp, rospy.Duration(0.5))

  pstamped = listener.transformPose(map_frame, detection.pose)

  #Add observation
  pos = numpy.array([pstamped.pose.position.x, pstamped.pose.position.y])
  tag_pos[detection.id][0] += pos/req_observations
  tag_pos[detection.id][1] += 1

  #Check if we have enough observations
  if tag_pos[detection.id][1] == req_observations:
    # Send to world state node
    pos = tag_pos[detection.id][0]
    add_object_service(name = "person_tag{}".format(detection.id), type = "person", x = pos[0], y = pos[1])
    # Remove from tracking
    tags.remove(detection.id)


def listener():
  global listener,pub,tags,tag_pos,req_observations,add_object_service,map_frame
  rospy.init_node('people_detector', anonymous=False)
  listener = tf.TransformListener()           # Note: init_node must happen first
  tags = []
  for s in rospy.get_param('~tags', "").split():
    tags.append(int(s))
  tag_pos = {}
  req_observations = float(rospy.get_param('~req_observations', 20))
  map_frame = rospy.get_param('~map_frame', "map")

  srv_name = 'world_state/add_object'
  rospy.wait_for_service(srv_name)
  add_object_service = rospy.ServiceProxy(srv_name, AddObject)

  rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tagCallback)
  rospy.spin()

if __name__ == "__main__":
  listener()

