#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Bool
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import time
import math

imu_health = False
wheel_encoder_health = False

def imu_callback(msg):

  imu_data = msg.orientation.x
  imu_health = False
  if(imu_data == True):
    imu_health = True

def wheel_encoder_callback(msg):

  wheel_encoder_data = msg.pose.pose.position.x
  wheel_encoder_health = False
  if(wheel_encoder_data == True):
    wheel_encoder_health = True

def health_monitor():
  start = time.time()
  health_monitor = False
  pub = rospy.Publisher('health_monitor', Bool, queue_size = 10)
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    if imu_health and wheel_encoder_health:
      health_monitor = True
      pub.publish(health_monitor)


rospy.init_node('Health Monitor')
rospy.Subscriber('/imu', Imu, imu_callback)
rospy.Subscriber('odom', Odometry, wheel_encoder_callback)
health_monitor()
rospy.spin()