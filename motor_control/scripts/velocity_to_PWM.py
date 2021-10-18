#! /usr/bin/env python

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist



class Vel2pwm:

    def __init__(self):
        self.lin_vel = 0
        self.current_vel = 0
        self.PWM = 0
    def cmd_vel_callback(self,twist):
        
        self.lin_vel = twist.linear.x
    
    def current_vel_callback(self,twist):
        
        self.current_vel = twist.linear.x

    def pwm_callback(self,data):
        self.PWM = data

    def main(self):
        
        rospy.init_node('Velocity_subscriber', anonymous=True)
        sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
        sub_current_vel = rospy.Subscriber("/state_estimations/veh_speed", Twist, current_vel_callback)
        sub_signal = rospy.Subscriber("/motor_control/motor_power", Int16, pwm_callback)
        
        pub = rospy.Publisher("/motor_control/motor_power", Int16,queue_size=10)

        while abs(self.current_vel - self.lin_vel) > 0.02: #this threshhold will have to change
            self.PWM = self.PWM + 1
            pub.publish(self.PWM)
            rospy.sleep(2) # 1 increment for every two seconds

        rospy.spin()

    if __name__== '__main__':
        main()       

