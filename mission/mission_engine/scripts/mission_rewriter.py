#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from threading import Lock
import simplejson

# Constants internal to node.
blocked = False
next_blocked = False
x = 0
dx = 0
debounced_false = False
debounced_true = False
debounce_count = 10
my_lock = Lock()
# Module state

def position_callback(data):
    global x
    x = data.pose.pose.position.x

def trajectory_callback(data):
    global dx
    dx = data.data

def obstacle_callback(data):
    global blocked, pub_mission_writer, x, dx, next_blocked
    global debounced_false, debounced_true, debounced_count
    global my_lock

    #if ((debounced_true and (data.data == "go")) or (debounced_false and (data.data == "stop") )
        #debounced_true = false
        #debounced_false = false
        #devounced_count = 10

    my_lock.acquire()
    next_blocked = blocked
    if((data.data == "stop") and (blocked== False)):
        # Going from unblocked to blocked; Nothing to do other than tracking state
        next_blocked = True
        txt = "Obstable detected! Absolute position: " + str(x)
        rospy.loginfo(txt)
        pub_ros2host_info.publish(txt)
    elif ((data.data == "go") and (blocked == True)):
        # going from blocked to unblocked; must rewrite program
        next_blocked = False
        txt = "Obstable removed! Rewrite and continue mission!"
        rospy.loginfo(txt)
        pub_ros2host_info.publish(txt)
        pub_mission_writer.publish("split " + str(x));
    else:
        # Nothing to do. Status quo.
        pass

    blocked = next_blocked
    my_lock.release()

def mission_rewriter():
    rospy.init_node('mission_rewriter', anonymous=True)

    rospy.Subscriber('string_lidar/obstacle', String, obstacle_callback)
    rospy.Subscriber('odom', Odometry, position_callback)
    rospy.Subscriber('trajectory', Float32, trajectory_callback)

    global pub_ros2host_info
    pub_ros2host_info = rospy.Publisher('ros2host_info', String, queue_size=10)

    global pub_mission_writer
    pub_mission_writer = rospy.Publisher('mission_rewrite', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    mission_rewriter()
