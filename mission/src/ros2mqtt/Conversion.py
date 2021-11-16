#! /usr/bin/env/ python
import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

import numpy as np

rospy.init_node("conversion")
pub_vel = rospy.Publisher("/string_velocity", String, queue_size=1)
pub_LIDAR = rospy.Publisher("/string_obstacle", String, queue_size=1)
pub_Distance = rospy.Publisher("/distance_to_obj", String, queue_size=1)
pub_lat = rospy.Publisher("/ros2host_lat", String, queue_size=1)
pub_long = rospy.Publisher("/ros2host_long", String, queue_size=1)


pub_static_speed = rospy.Publisher("/string_set_speed", String, queue_size=1)

set_static_speed = str(0.2)
pub_static_speed.publish(set_static_speed)


def callback_vel(msg):
    vel_x = msg.twist.twist.linear.x
    pub_vel.publish(str(vel_x))


def callback_lidar(msg):
    # distance_to_obj = msg.distance.sort()
    # print(distance_to_obj)

    obstacle = msg.data
    command = "stop"
    if obstacle:
       command = "go"
    else:
       command = "stop"

    pub_LIDAR.publish(command)
    # pub_Distance.publish(distance_to_obj)

def callback_gps(data):
    gps_lat = data.latitude
    gps_long = data.longitude
    pub_lat.publish(str(gps_lat))
    pub_long.publish(str(gps_long))



sub = rospy.Subscriber("/state_estimation", Odometry, callback_vel)
sub2 = rospy.Subscriber("/sensor/lidar/obstacles",Bool, callback_lidar)
sub3 = rospy.Subscriber("/sensor/gps/fix",NavSatFix,callback_gps)
rospy.spin()
