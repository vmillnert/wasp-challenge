#! /usr/bin/env python

import sys, pygame
import string
import rospy
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
    

# Fill background
# background = pygame.Surface(screen.get_size())
# background = background.convert()
# background.fill((250, 250, 250))

# Display some text
msg = """
Reading from the keyboard  and Publishing to Twist! 
---------------------------
Moving around: 
   u     i    o 
   j      k    l 
   m    ,    . 

For Holonomic mode (strafing), hold down the shift key: 
---------------------------
   U     I    O 
   J     K    L 
   M    <     > 

t : up (+z) 
b : down (-z) 

anything else : stop 

w/s : increase/decrease max speeds by 10% 
e/d : increase/decrease only linear speed by 10% 
r/f : increase/decrease only angular speed by 10% 

a: takeoff
z: land

x: MANUAL MODE  
c: AUTOMATIC MODE  
SPACE: PANIC BUTTON (Go to manual mode and STOP)  
q: quit  
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
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'w':(1.1,1.1),
		's':(.9,.9),
		'd':(1.1,1),
		'd':(.9,1),
		'r':(1,1.1),
		'f':(1,.9),
	      }





if __name__ == '__main__':

    # Start the publisher
    pub = rospy.Publisher('/bebop_teleop/cmd_vel', Twist, queue_size=1)
    command_pub = rospy.Publisher('/bebop_teleop/command', String, queue_size=10)
    
    rospy.init_node('teleop_twist_keyboard')
    
    pygame.init()

    size = width, height = 500, 600

    screen = pygame.display.set_mode(size)

    pygame.display.set_caption('Bebop Teleoperation')

    font = pygame.font.SysFont("comicsansms", 40)
    text = font.render("Teleoperation for Bebop", True, (10,10,10))
    textrect = text.get_rect()
    textrect.centerx = screen.get_rect().centerx
    # textrect.centery = screen.get_rect().centery


    font = pygame.font.SysFont("comicsansms", 20)

    screen.fill((255, 255, 255))
    for i in string.split(msg, '\n'):
        text = font.render(i[:-1], True, (10, 10, 10))
        textrect.centery += 17
        screen.blit(text, textrect)


        # # Blit everything to the screen
        # screen.blit(background, (0, 0))

        pygame.display.flip()

    speed = 0.5
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    command_msg = ""
    command = False
    shouldRun = True
    panic = False
    try:
        while shouldRun:
            for event in pygame.event.get():
        
                # Should quit
                if event.type == pygame.QUIT: sys.exit()

                # Key is pressed
                if event.type == pygame.KEYDOWN:
                    key  = pygame.key.name(event.key)
                    rospy.loginfo('%s is pressed' % key)
                    if key in moveBindings.keys():
                        x += moveBindings[key][0]
                        y += moveBindings[key][1]
                        z += moveBindings[key][2]
                        th += moveBindings[key][3]
                        command = True
                        command_msg = 'manual'
                    elif key in speedBindings.keys():
                        speed = speed*speedBindings[key][0]
                        turn = turn*speedBindings[key][1]
                        command = True
                        command_msg = 'manual'

                    elif key == 'q':
                        twist = Twist()
                        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                        pub.publish(twist)
                        shouldRun = False

                        break

                    elif key == 'space':
                        panic = True
                        rospy.loginfo('PANIC!!!!')
                        command = True
                        command_msg = 'manual'
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
                        command_msg = 'auto'
                        command = True

                # Key is released
                if event.type == pygame.KEYUP:
                    key = pygame.key.name(event.key)
                    print('%s is released' % key)
                    rospy.loginfo('%s is released' % key)
                    if key in moveBindings.keys():
                        x -= moveBindings[key][0]
                        y -= moveBindings[key][1]
                        z -= moveBindings[key][2]
                        th -= moveBindings[key][3]
                    if key == 'space':
                        rospy.loginfo('Stoped PANIC-mode')
                        panic = False


                if command:
                    command_pub.publish(command_msg)
                    command = False
                    print command_msg

                if not panic:
                    twist = Twist()
                    twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
                    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
                    pub.publish(twist)
                elif panic:
                    twist = Twist()
                    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                    pub.publish(twist)

    except Exception as e:
        rospy.loginfo(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        
