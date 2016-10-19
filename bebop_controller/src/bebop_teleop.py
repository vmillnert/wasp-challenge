#!/usr/bin/env python
import roslib;
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
t : up (+z)
b : down (-z)
anything else : stop
q/w : increase/decrease max speeds by 10%
e/r : increase/decrease only linear speed by 10%
s/d : increase/decrease only angular speed by 10%

x: MANUAL CONTROL (i.e. stop auto controller)
c: AUTOMATIC CONTROL
a: takeoff
z: Land
CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'w':(.9,.9),
		'e':(1.1,1),
		'r':(.9,1),
		's':(1,1.1),
		'd':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/bebop_teleop/cmd_vel', Twist, queue_size = 1)
        command_pub = rospy.Publisher('/bebop_teleop/command', String, queue_size=10)
	rospy.init_node('teleop_twist_keyboard')

	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print msg
                command_msg = ""
                command = False
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
                        elif key == 'a':
                            command_msg = "takeoff"
                            command = True
                        elif key == 'z':
                            command_msg = "land"
                            command = True
                        elif key == 'x':
                            command_msg = "manual"
                            command = True
                        elif key == 'c':
                            command_msg = "auto"
                            command = True
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

                        if command:
                            command_pub.publish(command_msg)
                            command = False
                            print command_msg

                        else:
                            twist = Twist()
                            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
                            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
                            pub.publish(twist)

	except rospy.ROSException:
            rospy.loginfo('Interrupt')

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
