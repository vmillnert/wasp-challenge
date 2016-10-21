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
from world.msg import TagRef

broadcaster = tf.TransformBroadcaster()

def locatedCallback(tagref):
  global listener

  try:
    # TODO: Could it be necessary to wait for transform?
    (trans_gt, rot_gt) = listener.lookupTransform(tagref.global_frame, tagref.tag_frame, tagref.pose.header.stamp)
    mat_map = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(trans_gt),
        tf.transformations.quaternion_matrix(rot_gt))

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
    
    sendTransform(tagref.global_frame, tagref.target_frame, trans, rot)
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    print("Transformation error")
    print(e)

def sendTransform(fromFrame, toFrame, trans,rot):
  global broadcaster
  broadcaster.sendTransform(trans,
                   rot,
                   rospy.Time.now(),
                   toFrame,
                   fromFrame)

def listener():
  global listener
  rospy.init_node('tagref', anonymous=False)

  listener = tf.TransformListener() # init_node needs to happen first

  rospy.Subscriber('tagref', TagRef, locatedCallback) # TODO: Name and type of topic

  rospy.spin()

if __name__ == "__main__":
  listener()

