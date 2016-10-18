#!/usr/bin/env python
#
# Listens to a rlaunch topic and executes roslauch with the received string argument. The topic name should be remapped to a name that
# makes sense for the project. For example:
#   rosnode rlaunch service.py rlaunch:=/myagentnname/rlaunch 
import rospy
from std_msgs.msg import String

import subprocess

def callback(data):
  print("Launching {}".format(data))
  subprocess.call("roslaunch {}".format(data))
    
def rlaunch_client():
  for k in os.environ:
    print(k)
  rospy.init_node('rlaunch', anonymous=True)
  rospy.Subscriber("rlaunch", String, callback)
  rospy.spin()

if __name__ == '__main__':
  rlaunch_client()
