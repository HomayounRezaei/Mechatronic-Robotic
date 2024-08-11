#!/usr/bin/python3

import rospy
from turtlesim.srv import Spawn, Kill

def spawn_turtle(x, y, theta, name):
    rospy.wait_for_service('spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        spawn_turtle(x, y, theta, name)
        return name
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

def kill_turtle(name):
    rospy.wait_for_service('kill')
    try:
        kill_turtle = rospy.ServiceProxy('kill', Kill)
        kill_turtle(name)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('draw_high_res_j_theta_with_turtles', anonymous=True)
   

    kill_turtle('turtle1')
   
    turtle_names = []
   
    j_positions = [ 
        (5.5, 9.5), (5.5, 9.0), (5.5, 8.5), (5.5, 8.0), (5.5, 7.5), (5.5, 7.0), (5.5, 6.5), (5.5, 6.0), (5.5, 5.5), (5.5, 5.0), (5.5, 4.5), (5.5, 4.0),

        (5.3, 3.7), (5, 3.5), (4.7, 3.3), (4.3, 3.2), (4, 3.1), (3.6, 3.1), (3.2, 3.2), (2.8, 3.3), (2.5, 3.5), (2.3, 3.7),

          (2.2, 3.9), (2.2, 4.1), (2.2,4.4),(2.2,4.7) ]  
   
    theta_positions = [
        (8.8,9),(8.3,8.8),(8,8.4),(7.84,8.1),(7.5,7.3),(7.42,7),(7.4,6.6),(7.4,5.8),(7.43,5.3),(7.55,4.8),(7.6,4.5),(7.82,4),(8,3.7),(8.38,3.5),(8.7,3.3),(8.9,3.3),(9.26,3.4),(9.53,3.6),(9.73,3.85),(10,4.3),(10.1,4.8),(10.3,5.4),(10.3,6.2),(10.3,6.8),(10.17,7.3),(9.97,8),(9.74,8.3),(9.53,8.6),(9.22,8.8),(8.95,8.9)
        ,(7.8,6.2),(8,6.2),(8.2,6.2),(8.4,6.2),(8.6,6.2),(8.8,6.2),(9,6.2),(9.3,6.2),(9.5,6.2),(9.7,6.2)
    ]
   

    for i, (x, y) in enumerate(j_positions):
        name = f'turtle_j{i+1}'
        if spawn_turtle(x, y, 0, name):
            turtle_names.append(name)
   

    for i, (x, y) in enumerate(theta_positions):
        name = f'turtle_theta{i+1}'
        if spawn_turtle(x, y, 0, name):
            turtle_names.append(name)
   

    rospy.set_param('/turtle_names', turtle_names)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass