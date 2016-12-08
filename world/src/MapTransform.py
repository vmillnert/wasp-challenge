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
  global observation_history # A dictionary of last observation for every landmark to publish
  global n_samples # Number of samples for median filter
  try:
    #rospy.loginfo('Request time: '+ str(tagref.pose.header.stamp))
    listener.waitForTransform(tagref.global_frame, tagref.tag_frame,
                              tagref.pose.header.stamp, rospy.Duration(2))
    # CHANGE tagref.pose.header.stamp -> rospy.Time.now() ???
    (trans_gt, rot_gt) = listener.lookupTransform(tagref.global_frame,
                                                  tagref.tag_frame, tagref.pose.header.stamp)
    # CHANGE tagref.pose.header.stamp -> rospy.Time.now() ???
    mat_map = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(trans_gt),
        tf.transformations.quaternion_matrix(rot_gt))

    listener.waitForTransform(tagref.target_frame, tagref.pose.header.frame_id,
                              tagref.pose.header.stamp, rospy.Duration(0.5))
    # CHANGE tagref.pose.header.stamp -> rospy.Time.now() ???

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
    if not key in observation_history:
        observation_history[key] = []
    assert(len(observation_history[key]) <= n_samples)
    if len(observation_history[key]) == n_samples:
        observation_history[key].pop(0)
    observation_history[key].append((trans,rot))


    if len(observation_history[key]) == n_samples:
      trans_list = map(lambda x : x[0] , observation_history[key])
      rot_list = map(lambda x : x[1] , observation_history[key])

      if not key in transforms:
        # TODO: Now using known tag location, perhaps use an initial average
        trans_0 = trans_gt
        rot_0   = rot_gt
      else:
        trans_0 = transforms[key][2]
        rot_0 = transforms[key][3]

      trans_norm = map(lambda x: (numpy.linalg.norm(trans_0-x), x) , trans_list)
    
      # compute the 'norms' (i.e. distance) between the quaternions
      # the inner product between two quaternions q1 and q2 is cos(theta/2)
      # where theta is the smalles arc between them. Hence the
      # 'distance' between the two quaternions is defined as
      # 2*acos(dot(q1,q2))
      rot_norm = map(lambda x: (2*numpy.arccos(numpy.dot(rot_0, x)), x), rot_list)
    
      trans_norm = sorted(trans_norm, key=lambda x: x[0])
      rot_norm = sorted(rot_norm, key=lambda x: x[0])
      i = int(len(trans_norm)/2)
    
      transforms[key] = (tagref.global_frame, tagref.target_frame, trans_norm[i][1], rot_norm[i][1], tagref.pose.header.stamp)
      # CHANGE tagref.pose.header.stamp -> rospy.Time.now() ???
    lock.release()

  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    print("Transformation error")
    print(e)

# Publishes the transform
def sendTransform(fromFrame, toFrame, trans,rot, time):
  global broadcaster
  #broadcaster.sendTransform(trans, rot, rospy.Time.now(), toFrame, fromFrame)
  broadcaster.sendTransform(trans, rot, time, toFrame, fromFrame)

# Periodically broadcasts the transform
def broadcastThread():
  global transforms, lock
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    lock.acquire()
    for k,v in transforms.iteritems():
      rospy.loginfo
      # sendTransform(v[0], v[1], v[2], v[3], v[4])
      sendTransform(v[0], v[1], v[2], v[3], rospy.Time.now())
    lock.release()
    rate.sleep()

# Create the tag reference listener
def startMapTransform():
  global listener       # Used by locatedCallback
  global broadcaster    # Used by sendTransform
  global transforms     # Published periodically
  global lock           # Synchronize locatedCallback and broadcast thread
  global observation_history # A dictionary of last observation for every landmark to publish
  global n_samples # Number of samples for median filter
  lock = Lock()
  n_samples = 31
  transforms = {}
  broadcaster = tf.TransformBroadcaster()
  observation_history = {}
  rospy.init_node('tagref', anonymous=False)
  listener = tf.TransformListener()           # Note: init_node must happen first
  Thread(target=broadcastThread).start()
  rospy.Subscriber('tagref', TagRef, locatedCallback)
  rospy.spin()

if __name__ == "__main__":
  startMapTransform()
