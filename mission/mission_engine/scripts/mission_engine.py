#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import constants as c
import simplejson

# Constants internal to node.
BUSY = 1
WAITING = 0

# Module state

state = WAITING
program = None
PC = 0 # program counter

def load_program(mission_name):
    rospy.loginfo('loading %s', mission_name)
    f = open(c.MISSION_HOME + mission_name,'r')
    program = simplejson.load(f)
    f.close()
    rospy.loginfo('loaded %d commands', len(program))

def run_program():
    while (PC <= len(program)):
        # Main command interpretation 
        interpret(program[PC])
        PC++
        

def interpret(inst):
    if (inst[0]== c.MOVE):
        # publish to trajectory
        pass
    elif (inst[0] == c.LOGSTART):
        # publish to rosbag_begin
        pass
    else 
        rospy.loginfo('Illegal instruction in mission')


def callback(data):
    tokens = data.data.split()
    if(state == WAITING):
        state = BUSY
        rospy.loginfo('received %s when ready', data.data)
        if(tokens[0] == "load"):
            load_program(tokens[1])

        elif(tokens[0] == "purge"):
            program = None
            PC = 0
        elif(tokens[0] == "execute"):
            if(program == None):
                rospy.logInfo('ERROR: No mission loaded. Load mission before attempting to execute.')
            else 
                run_program()
        elif(tokens[0] == "singlestep"):
            if(program == None):
                rospy.logInfo('ERROR: No mission loaded. Load mission before attempting to execute.')
            else 
                single_step() 
        elif(tokens[0] == "RTS"):
            pass
        else:
            rospy.loginfo("Illegal instruction %s", data.data)
        state = WAITING
    elif (state == BUSY):
        pass 
    
def mission_engine():
    rospy.init_node('mission_engine', anonymous=True)

    rospy.Subscriber('mission_cmds', String, callback)

    rospy.spin()

if __name__ == '__main__':
    mission_engine()
