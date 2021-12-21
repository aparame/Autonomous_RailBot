#! /usr/bin/env python

import rospy, math, numpy as np 

from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Float32

IDLE = 'IDLE'
RUNNING = 'RUNNING'
MAX_PWM = 170
MIN_PWM = 84
MAX_VEL = 0.5 #bounding the velocity to 0.5 m/s

rospy.init_node('Motor_Control_Compat', anonymous=True)

class MotorControl():

    def __init__(self):
        self.trajectory = 0
        self.obstacle = False
        self.state = IDLE
        self.feedback_gain = 0.1
        self.current_x_pose = 0
        self.current_y_pose = 0
        self.current_x_vel = 0
        self.current_y_vel = 0
        self.cmd_vel = 0
        self.pwm = 127
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
    
    def pose_callback(self, msg):
        self.current_x_pose = msg.pose.pose.position.x
        self.current_y_pose = msg.pose.pose.position.y

    def pseudopose_callback(self, msg):
        self.current_x_pose = msg.data
        self.current_y_pose = 0

    def velocity_callback(self,msg):
        self.current_x_vel = msg.linear.x
        self.current_y_vel = msg.linear.y

    def trajectory_callback(self, msg):
        self.trajectory = msg.data
        self.state = RUNNING
        rospy.loginfo('Starting Motors to move %d meters', self.trajectory)
    
    def obstacle_callback(self, msg):
        self.obstacle = msg.data
        if(self.obstacle):
            donepub.publish(True)
            self.cmd_vel = 0
            self.trajectory = 0

    def timer_callback(self,timer):
        if(self.state == IDLE):
            return
        rospy.loginfo('timer interrupt while running')
        self.feedback_controller() 
        self.acceleration_bounding()

    def feedback_controller(self):
        dx = self.trajectory - self.current_x_pose
        dy = 0 - self.current_y_pose

        rho = math.sqrt(dx**2 + dy**2)
        if(rho > 0.1):
            self.cmd_vel = self.feedback_gain * rho # New velocity setpoint proportional to position error
            rospy.loginfo('new setpoint velocity: %5.2f',self.cmd_vel)
            
            # SETPOINT truncated because it should never exceed MAX_VEL
            if(self.cmd_vel >= MAX_VEL):
                self.cmd_vel = MAX_VEL
                rospy.loginfo('setpoint velocity capped to : %5.2f',self.cmd_vel)
        else:
            self.cmd_vel = 0
            self.state = IDLE
            rospy.loginfo('reached target position. Command done.')
            donepub.publish(True) # Done executing the command
    
    # function that iteratively accelerates to hit the velocity set-point
    def acceleration_bounding(self):
        # If IDLE, the PWM should be logical zero (which corresponds to PWM=127)
        if(self.state == IDLE):
            self.pwm = 127
            pub.publish(self.pwm)
            return    

        diff = self.current_x_vel - self.cmd_vel
        if(abs(diff) > 0.02): #velocity error threshold
            if(diff > 0): # Actual velocity greater than setpoint, reduce PWM
                self.pwm -= 1
                if(self.pwm < MIN_PWM):
                    self.pwm = MIN_PWM
            elif(diff < 0): # Actual velocity less than setpoint, increase PWM
                self.pwm += 1
                if(self.pwm > MAX_PWM):
                    self.pwm = MAX_PWM
            
            rospy.loginfo('pwm: %d',self.pwm)
            pub.publish(self.pwm)
        else:
            rospy.loginfo('reached target velocity %5.2f',self.current_x_vel)

            
    def main(self):
        rospy.Subscriber('/state_estimations/odometry', Odometry, self.pose_callback) #need to check if the topic is correct
        rospy.Subscriber('/testX', Float32, self.pseudopose_callback) # fake odometry for testing. Only X position 
        rospy.Subscriber('/trajectory', Float32, self.trajectory_callback) # need to check the topic name and type. Expecting a 32 bit signed Int for the mission
        rospy.Subscriber('/obstacle', Bool, self.obstacle_callback) #need to check the topic name and type. Expecting a boolean
        rospy.Subscriber('/state_estimations/velocity', Twist, self.velocity_callback) #need to validate the topic name and msg type

        global pub 
        pub = rospy.Publisher('/velocity', Int32, queue_size = 10)
        global donepub 
        donepub = rospy.Publisher('/cmd_done', Bool, queue_size = 10)


mc = MotorControl()
mc.main()
rospy.spin()
