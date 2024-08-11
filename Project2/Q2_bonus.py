#!/usr/bin/python3

import rospy
import pygame
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill

def spawn_turtle(x, y, theta, name):
    rospy.wait_for_service('spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        spawn_turtle(x, y, theta, name)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def kill_turtle(name):
    rospy.wait_for_service('kill')
    try:
        kill_turtle = rospy.ServiceProxy('kill', Kill)
        kill_turtle(name)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def draw_letters():

    kill_turtle('turtle1')
   
    turtle_names_j = []
   

    j_positions = [
        (5.5, 9.5), (5.5, 9.0), (5.5, 8.5), (5.5, 8.0), (5.5, 7.5), (5.5, 7.0),
        (5.5, 6.5), (5.5, 6.0), (5.5, 5.5), (5.5, 5.0), (5.5, 4.5), (5.5, 4.0),
        (5.3, 3.7), (5, 3.5), (4.7, 3.3), (4.3, 3.2), (4, 3.1), (3.6, 3.1),
        (3.2, 3.2), (2.8, 3.3), (2.5, 3.5), (2.3, 3.7), (2.2, 3.9), (2.2, 4.1),
        (2.2, 4.4), (2.2, 4.7)
    ]  
   

    for i, (x, y) in enumerate(j_positions):
        name = f'turtle_j{i+1}'
        spawn_turtle(x, y, 0, name)
        turtle_names_j.append(name)
   

    rospy.set_param('/turtle_names_j', turtle_names_j)

def handle_key_presses():

    if rospy.has_param('/turtle_names_j'):
        turtle_names_j = rospy.get_param('/turtle_names_j')
        if not turtle_names_j:
            rospy.logwarn("No turtles found for letter J in parameter server!")
            return
    else:
        rospy.logwarn("No turtles found in parameter server!")
        return


    publishers = {name: rospy.Publisher(f'/{name}/cmd_vel', Twist, queue_size=10) for name in turtle_names_j}
   
    pygame.init()
    screen = pygame.display.set_mode((100, 100))
    pygame.display.set_caption("Turtle Control")

    rate = rospy.Rate(10)  # 10 Hz
    twist = Twist()

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    twist.linear.x = 2.0
                    twist.angular.z = 0.0
                elif event.key == pygame.K_DOWN:
                    twist.linear.x = -2.0
                    twist.angular.z = 0.0
                elif event.key == pygame.K_LEFT:
                    twist.linear.x = 0.0
                    twist.angular.z = 2.0
                elif event.key == pygame.K_RIGHT:
                    twist.linear.x = 0.0
                    twist.angular.z = -2.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                for pub in publishers.values():
                    pub.publish(twist)

            if event.type == pygame.KEYUP:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                for pub in publishers.values():
                    pub.publish(twist)

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('turtle_key_control', anonymous=True)
        draw_letters()
        handle_key_presses()
    except rospy.ROSInterruptException:
        pass