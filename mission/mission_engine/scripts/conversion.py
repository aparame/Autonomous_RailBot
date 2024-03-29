#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# import numpy as np

def callback_vel(msg):
    vel_x = msg.twist.twist.linear.x
    pub_vel.publish(str(vel_x))

def callback_lidar(msg):
    # distance_to_obj = msg.distance.sort()
    # print(distance_to_obj)

    obstacle = msg.data
    command = "stop"
    if not obstacle:
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

def conversion():
    rospy.init_node("conversion", anonymous=True)
    global pub_vel
    global pub_LIDAR
    global pub_Distance
    global pub_lat
    global pub_long

    pub_vel = rospy.Publisher("/string_velocity", String, queue_size=1)
    pub_LIDAR = rospy.Publisher("/string_lidar/obstacle", String, queue_size=1)
    pub_Distance = rospy.Publisher("/distance_to_obj", String, queue_size=1)
    pub_lat = rospy.Publisher("/ros2host_lat", String, queue_size=1)
    pub_long = rospy.Publisher("/ros2host_long", String, queue_size=1)

    pub_static_speed = rospy.Publisher("/string_set_speed", String, queue_size=1)
    set_static_speed = str(0.2)
    pub_static_speed.publish(set_static_speed)

    rospy.Subscriber("/state_estimation", Odometry, callback_vel)
    rospy.Subscriber("/sensor/lidar/obstacle",Bool, callback_lidar)
    rospy.Subscriber("/sensor/gps/fix",NavSatFix,callback_gps)
    rospy.spin()

if __name__ == '__main__':
    conversion()
