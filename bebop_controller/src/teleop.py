#! /usr/bin/env python

import sys, pygame
import string
import rospy
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import thread
    

# Fill background
# background = pygame.Surface(screen.get_size())
# background = background.convert()
# background.fill((250, 250, 250))

# Display some text
msg = """
TO CONTROL THE DRONE, THIS WINDOW MUST BE IN FOCUS   

Reading from the keyboard and publishing to drone!   
---------------------------  
Moving around: 

Use the arrows:

Up-arrow:     Forward   
Left-arrow:    Rotate left   
Right-arrow:  Rotate right   
Down-arrow:  Backward   

, : up (+z)  
. : down (-z)  

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
		'up':(1,0,0,0),
		'left':(0,0,0,1),
		'right':(0,0,0,-1),
		'down':(-1,0,0,0),
		',':(0,0,1,0),
		'.':(0,0,-1,0),
	       }

speedBindings={
		'w':(1.1,1.1),
		's':(.9,.9),
		'd':(1.1,1),
		'd':(.9,1),
		'r':(1,1.1),
		'f':(1,.9),
	      }


def run():
    # Start the publisher
    #TODO: change the namespacing here!
    name = rospy.names.get_namespace()[0:-1]
    rospy.loginfo('name: %s', name)
    pub = rospy.Publisher(name+'/bebop_teleop/cmd_vel', Twist, queue_size=1)
    command_pub = rospy.Publisher(name+'/bebop_teleop/command', String, queue_size=10)

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
                # IMPROVE THIS ONE
                if event.type == pygame.QUIT: sys.exit()

                # Key is pressed
                if event.type == pygame.KEYDOWN:
                    key  = pygame.key.name(event.key)
                    #rospy.loginfo('%s is pressed' % key)
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
                    #rospy.loginfo('%s is released' % key)
                    if key in moveBindings.keys():
                        x -= moveBindings[key][0]
                        y -= moveBindings[key][1]
                        z -= moveBindings[key][2]
                        th -= moveBindings[key][3]
                    if key == 'space':
                        rospy.loginfo('Stoped PANIC-mode')
                        panic = False


                if command:
                    if command_msg == 'land':
                        command_pub.publish('manual')
                        rospy.sleep(0.5)
                    
                    command_pub.publish(command_msg)
                    command = False
                    print command_msg

                if not panic:
                    twist = Twist()
                    twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
                    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
                    # rospy.loginfo('Velocity: \n linear: x: %.2f\n z: %.2f \n angular: \n z: %.2f',
                    #               twist.linear.x,
                    #               twist.linear.z,
                    #               twist.angular.z)
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
        


if __name__ == '__main__':
    rospy.init_node('teleop_twist_keyboard')
    thread.start_new_thread(run, ())
    rospy.spin()
