#! /usr/bin/env python

import rospy, math, numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


rospy.init.node('motor_control', anonymous=True)


class VelocityPublisher(): #velocity publisher
    def __init__(self, topic):
        
        self.cmd_vel = rospy.Publisher(topic,Twist,queue_size=10)
        rospy.sleep(0.1)

    def move(self, linear_vel = 0.0):
        msg = Twist()
        #linear_vel = acceleration_bound(linear_vel) bounding the acceleration here
        msg.linear.x = linear_vel
        self.cmd_vel.publish(msg)


class Odometryreader():
    def __init__(self, topic):
        self.pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg): #extracting x, y and heading angle from state estimation
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.pose['x'], self.pose['y']))
        (_,_, self.pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    
    def subscribe(self): #odometry subscriber 
        self.subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self): #saving the trajectory of the railpbot 
        np.save('Trajectory',self.trajectory)
        self.subscriber.unregister()

k_rho = 0.1 #feedback gain

def go_to_next_way_point(cmd_x, cmd_y, constant_vel = None):
    rho = float("inf")
    while (rho > 0.01): #position feedback controller
        dx = cmd_x - odometry.pose['x'] 
        dy = cmd_y - odometry.pose['y'] 
        rho = np.sqrt(dx**2 + dy**2) #differential euclidian distance
        v = k_rho * rho
        if(constant_vel): # velocity bound
            v = v * constant_vel / abs(v)
        velcoity.move(v)
        rospy.sleep(0.1)
         

velcoity = VelocityPublisher('/cmd_vel')
odometry = Odometryreader('/state_estimations/odometry')

#sample inputs in meters 
x = 2
y = 0  # will be input from Dr. Mithuna mission planning code

go_to_next_way_point(x,y,constant_vel=0.2)

velcoity.move(0,0)
odometry.unregister()

# error = differential GPS coords 


