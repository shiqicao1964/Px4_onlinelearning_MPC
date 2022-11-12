#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped ,Pose, Twist

if __name__ == "__main__":
    rospy.init_node("reference_publisher")
    rate = rospy.Rate(1)
    reference_pub = rospy.Publisher("reference", Twist, queue_size=1)
    reference_pub_2 = rospy.Publisher("reference2", Pose, queue_size=1)
    msg = Twist()
    msg_2 = Pose()
    msg_2.position.x = 1.2
    msg_2.position.y = 2.2
    msg_2.position.z = 3.2
    msg_2.orientation.x = 0.1
    msg_2.orientation.y = 0.2
    msg_2.orientation.z = 0.3
    msg_2.orientation.w = 0.4
    msg.linear.x = 1
    msg.linear.y = 2
    msg.linear.z = 3
    msg.angular.x = 4
    msg.angular.y = 5
    msg.angular.z = 6

    while(1):
        reference_pub.publish(msg)
        reference_pub_2.publish(msg_2)
        rate.sleep()
