#!/usr/bin/env python
import rospy
import tf
from world.msg import TagRef
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

global pub,tags,agent_frame

def tagCallback(data):
  global pub,tags,agent_frame
  for detection in data.detections:
    if detection.id in tags:
      tagref = TagRef()
      tagref.global_frame = "map"
      tagref.tag_frame = "landmark"+str(detection.id)
      tagref.target_frame = agent_frame
      tagref.pose = detection.pose
      q = detection.pose.pose.orientation
      p = detection.pose.pose.position
      e =tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w)) 
      rospy.loginfo("Landmark {} detected at {:5.2f},{:5.2f},{:5.2f} {:5.2f},{:5.2f},{:5.2f} for {}".format(
        detection.id, p.x, p.y, p.z, e[0], e[1], e[2],
        agent_frame))
      pub.publish(tagref)


def listener():
  global listener,pub,tags,agent_frame
  rospy.init_node('landmark_detector', anonymous=True)

  # Publishes to the tagref handler which repositions the drone
  pub = rospy.Publisher('tagref', TagRef, queue_size=10)

  agent_frame = rospy.get_param('~agent_frame')
  if not agent_frame:
    raise Exception("Must specify an agent frame")

  tags = []
  for s in rospy.get_param('~tags', "").split():
    tags.append(int(s))

  rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tagCallback) 
  rospy.spin()

if __name__ == "__main__":
  listener()

