#!/usr/bin/env python

import rospy
import tf
from world.msg import TagRef

broadcaster = tf.TransformBroadcaster()

def locatedCallback(tagref):
  print(tagref)
  print("TODO: Implement the locatedCallback")
  sendTransform(0, 0, 0, 0)

def sendTransform(dx, dy, dz, theta):
  br = tf.TransformBroadcaster()
  br.sendTransform((dx, dy, dz),
                   tf.transformations.quaternion_from_euler(0, 0, theta),
                   rospy.Time.now(),
                   "odom",
                   "map")

def listener():
  rospy.init_node('tagrefListener', anonymous=False)
  rospy.Subscriber('tagref', TagRef, locatedCallback) # TODO: Name and type of topic
  rospy.spin()

if __name__ == "__main__":
  listener()
