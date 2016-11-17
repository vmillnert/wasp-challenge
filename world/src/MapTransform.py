#!/usr/bin/env python
#
# When some agent identifies a tag it publishes this information on the topic
# tagref. The tag has a reference in the map tree. Origin is a static
# [tranformation] parent to the tag frame (for example a turtlebot odom or
# a box). The dotted line from map to odom is the link that needs to be updated
# (possibly not yet existing).
#
#                      map
#                       |
#                .......-----------------...
#                |             |      |
#              odom          origin  ...
#                |             |
#              agent          ...
#                |             |
#  tag <-- (o) sensor      ---------
#                          |       |
#                         ...   tag_frame
#
#
# The update proceeds as follows:
#     1. Get the transform map->tag_frame
#     2. Get the tag pose expressed in odom frame.
#     3. Find a transform map->odom such that map->tag = map->tag_frame 
#     4. Update the transform map->odom
#
import rospy
import tf
import math
import numpy
from threading import Thread,Event,Lock
from world.msg import TagRef

# This callback is invoked when a tag (landmark) is spotted
def locatedCallback(tagref):
  global listener   # Created in listener()
  global transforms  # A dictionary of transforms to publish

  try:
    listener.waitForTransform(tagref.global_frame, tagref.tag_frame, tagref.pose.header.stamp, rospy.Duration(0.5))
    (trans_gt, rot_gt) = listener.lookupTransform(tagref.global_frame, tagref.tag_frame, tagref.pose.header.stamp)
    mat_map = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(trans_gt),
        tf.transformations.quaternion_matrix(rot_gt))

    listener.waitForTransform(tagref.target_frame, tagref.pose.header.frame_id, tagref.pose.header.stamp, rospy.Duration(0.5))
    pstamped = listener.transformPose(tagref.target_frame, tagref.pose)

    mat_odom = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix((
          pstamped.pose.position.x,
          pstamped.pose.position.y,
          pstamped.pose.position.z)),
        tf.transformations.quaternion_matrix((
          pstamped.pose.orientation.x,
         pstamped.pose.orientation.y,
          pstamped.pose.orientation.z,
          pstamped.pose.orientation.w
        )))

    mat_odom_inv = tf.transformations.inverse_matrix(mat_odom)
    mat_map_to_odom = numpy.dot(mat_map, mat_odom_inv)
    trans = tf.transformations.translation_from_matrix(mat_map_to_odom)
    rot = tf.transformations.quaternion_from_matrix(mat_map_to_odom)
   
    lock.acquire() 
    key=tagref.global_frame+","+tagref.tag_frame+","+tagref.target_frame
    transforms[key] = (tagref.global_frame, tagref.target_frame, trans, rot)
    lock.release()

  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    print("Transformation error")
    print(e)

# Publishes the transform
def sendTransform(fromFrame, toFrame, trans,rot):
  global broadcaster
  broadcaster.sendTransform(trans, rot, rospy.Time.now(), toFrame, fromFrame)

# Periodically broadcasts the transform
def broadcastThread():
  global transforms, lock
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    lock.acquire()
    for k,v in transforms.iteritems():
      sendTransform(v[0], v[1], v[2], v[3])
    lock.release()
    rate.sleep()

# Create the tag reference listener
def startMapTransform():
  global listener       # Used by locatedCallback
  global broadcaster    # Used by sendTransform
  global transforms     # Published periodically
  global lock           # Synchronize locatedCallback and broadcast thread

  lock = Lock()
  transforms = {}
  broadcaster = tf.TransformBroadcaster()
  rospy.init_node('tagref', anonymous=False)
  listener = tf.TransformListener()           # Note: init_node must happen first
  Thread(target=broadcastThread).start()
  rospy.Subscriber('tagref', TagRef, locatedCallback)
  rospy.spin()

if __name__ == "__main__":
  startMapTransform()
