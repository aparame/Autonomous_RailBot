#! /usr/bin/env/ python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np

rospy.init_node("conversion")
pub = rospy.Publisher("/converter", String, queue_size=1)


def callback(msg):
    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    dist_x = msg.pose.pose.position.x
    dist_y = msg.pose.pose.position.y
    values = np.array([vel_x, vel_y, dist_x, dist_y])

    for i in values:
        pub.publish(str(i))


sub = rospy.Subscriber("/state_estimation", Odometry, callback)
rospy.spin()
