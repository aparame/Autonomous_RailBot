#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import constants as c
import simplejson

# Constants internal to node.
blocked = False 
# Module state

def position_callback(data):
    global x 
    x = data.data

def obstacle_callback(data):
    global blocked, pub_mission_writer
    next_blocked = False
    if((data.data == "stop") and (blocked== False)):
        # Going from unblocked to blocked; Nothing to do other than tracking state
        next_blocked = True
    elif ((data.data == "go") and (blocked == True)):
        # going from blocked to unblocked; must rewrite program
        next_blocked = False
        pub_mission_writer.publish("split " + str(x));
    else:
        # Nothing to do. Status quo.
        pass

    blocked = next_blocked
        

def mission_rewriter():
    rospy.init_node('mission_rewriter', anonymous=True)

    rospy.Subscriber('string_lidar/obstacle', String, obstacle_callback)
    rospy.Subscriber('pose/pose/position/x', Float32, position_callback)

    global pub_mission_writer
    pub_mission_writer = rospy.Publisher('mission_rewrite', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    mission_rewriter()
