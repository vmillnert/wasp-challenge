#!/usr/bin/env python
#
# Listens to a rlaunch topic and executes roslaunch with the received string argument. The special command KILL kills all previously started processes. The topic name should be remapped to a name that
# makes sense for the project. For example:
#   rosnode rlaunch service.py rlaunch:=/myagentnname/rlaunch
#
# WARNING!!! Do not spread this service as is outside a contained environment! It uses the shell for execution with poses a security risk!
#
import rospy
from std_msgs.msg import String

import os
import subprocess
import psutil

global children
children = []

def callback(data):
  global children
  if data.data == "KILL":
    for p in children:
      print("Killing {}".format(p.pid))
      subprocess.call(["/usr/bin/pkill", "-TERM", "-P", str(p.pid)])
    children = []
  else:
    cmd = "roslaunch {}".format(data.data)
    print(cmd)
    children.append(subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True))
    
def rlaunch_client():
  rospy.init_node('rlaunch', anonymous=True)
  rospy.Subscriber("rlaunch", String, callback)
  print("Ready for command")
  rospy.spin()

if __name__ == '__main__':
  rlaunch_client()

