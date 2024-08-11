#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

def rotate_turtles():
    rospy.init_node('rotating_node', anonymous=True)
   

    if rospy.has_param('/turtle_names'):
        turtle_names = rospy.get_param('/turtle_names')
        rospy.loginfo(f"Found turtles: {turtle_names}")
    else:
        rospy.logwarn("No turtles found in parameter server!")
        return

    publishers = {name: rospy.Publisher(f'/{name}/cmd_vel', Twist, queue_size=10) for name in turtle_names}
   
    rate = rospy.Rate(10)  # 10 Hz
    twist = Twist()
    twist.angular.z = 1.0  # Adjust the angular velocity as needed

    rospy.loginfo("Rotating all turtles...")

    while not rospy.is_shutdown():
        for name, pub in publishers.items():
            pub.publish(twist)
            rospy.loginfo(f"Publishing to {name}")
        rate.sleep()

if __name__ == '__main__':
    try:
        rotate_turtles()
    except rospy.ROSInterruptException:
        pass