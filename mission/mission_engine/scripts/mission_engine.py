#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import simplejson

class constants:
    # Command Definitions
    MOVE = 1
    LOGSTART = 2
    LOGEND = 3

    # Project-wide directories
    MISSION_HOME = "../missions/"

    # LIDAR THRESHOLD
    OBSTACLE_THRESHOLD = 3.3

# Constants internal to node.
WAITING = 0
BUSY = 1
EXECUTING = 2
BLOCKED = 3
IDLE = 4
c = constants()
# Module state

state = WAITING
execstate = IDLE
program = None
PC = 0 # program counter
autostep = False
cumulative_x = 0 #cumulative distance traversed

def load_program(mission_name):
    global program
    rospy.loginfo('loading %s', c.MISSION_HOME + mission_name)
    f = open(c.MISSION_HOME + mission_name,'r')
    program = simplejson.load(f)
    f.close()
    rospy.loginfo('loaded %d commands', len(program))

def run_program():
    # Main command interpretation
    interpret(program[PC])
    global autostep
    autostep = True

def single_step():
    global autostep
    rospy.loginfo('State (%d)', state)
    if(execstate == EXECUTING):
        rospy.loginfo('Ignoring single step command; previous command (%d) still not complete.', PC)
    else:
        interpret(program[PC])
        autostep = False

def interpret(inst):
    global execstate, cumulative_x
    if (inst[0]== c.MOVE):
        # publish to trajectory
        rospy.loginfo('started command %d: Move %5.2f',PC,inst[1])
        cumulative_x += float(inst[1])
        pub_mission_active.publish(True)
        execstate = EXECUTING
        pub_trajectory.publish(inst[1])

    elif (inst[0] == c.LOGSTART):
        # publish to rosbag_begin
        pass
    else :
        rospy.loginfo('Illegal instruction in mission')

def rewrite_callback(data):
    global program, PC, execstate,cumulative_x
    if (program == None):
        rospy.loginfo('Weird -- No program loaded or executing; yet safety-monitor wants to rewrite the program. Should not happen - please investigate and debug')
        return
    elif (execstate == IDLE):
        rospy.loginfo('Weird -- No command executing; yet safety-monitor wants to rewrite the program. Should not happen - please investigate and debug')
        return
    tokens = data.data.split()
    if (tokens[0] == "split"):
        splitPoint = float(tokens[1])
        remainder = cumulative_x - splitPoint
        partial_leg = splitPoint - (cumulative_x - program[PC][1])
        #correct the cumulative_x because only the partial leg was actually covered
        cumulative_x = cumulative_x -program[PC][1] + partial_leg
        rospy.loginfo("Splitting %5.2f into %5.2f and %5.2f",program[PC][1],splitPoint,remainder)
        rospy.loginfo(str(program))
        rospy.loginfo(str(PC))
        # delete old command
        rospy.loginfo("Old program: " + str(program))
        prenode = [program[PC][0], partial_leg]
        postnode = [program[PC][0], remainder]
        del program[PC]
        if(PC < len(program)):
            program.insert(PC, prenode)
            program.insert(PC+1,postnode)
        else:
            program.append(prenode)
            program.append(postnode)
        PC+=1
        rospy.loginfo("Rewritten new program: " + str(program))
        if (autostep):
            rospy.loginfo('starting command number %d in new program',PC)
            interpret(program[PC])
        else:
            rospy.loginfo('Waiting to continue execution (or singlestep) at new command %d',PC)
            execstate = IDLE
    else:
        #other program rewrite cases. For future use cases.
        pass

def cmd_done_callback(data):
    global PC, execstate, program
    if (program == None):
        rospy.loginfo('Weird -- No program loaded or executing; yet bot claims commands are done executing. Should not happen - please investigate and debug')
        return
    elif (execstate == IDLE):
        rospy.loginfo('Weird -- No command executing; yet bot claims commands are done executing. Should not happen - please investigate and debug')
        return
    elif (data.data == False):
        rospy.loginfo('Weird -- cmd_done == False. Should not happen - please investigate and debug')
        return
    rospy.loginfo('Done with command number %d',PC)
    PC +=1
    if(PC == len(program)):
        pub_mission_active.publish(False)
        program = None
        execstate = IDLE
        PC = 0
        rospy.loginfo('Done with mission. Reload next mission to continue.')
    else:
        if (autostep):
            rospy.loginfo('starting command number %d',PC)
            interpret(program[PC])
        else:
            rospy.loginfo('Waiting to continue execution (or singlestep) at command %d',PC)
            execstate = IDLE

def host2ros_callback(data):
    global state, program, PC, execstate,cumulative_x
    tokens = data.data.split()
    if(state == WAITING):
        state = BUSY
        rospy.loginfo('received %s when ready', data.data)
        if(tokens[0] == "load"):
            if len(tokens) == 1:
                rospy.loginfo('Please give me a mission to load')
            else:
                try:
                    load_program(str(tokens[1]))
                except:
                    rospy.loginfo("Cannot load mission file "+str(tokens[1]))
        elif(tokens[0] == "purge"):
            program = None
            PC = 0
            cumulative_x = 0
            execstate = IDLE
            rospy.loginfo('Purged loaded program')
        elif(tokens[0] == "execute"):
            if(program == None):
                rospy.loginfo('ERROR: No mission loaded. Load mission before attempting to execute.')
            else:
                run_program()
        elif(tokens[0] == "singlestep"):
            if(program == None):
                rospy.loginfo('ERROR: No mission loaded. Load mission before attempting to execute.')
            else:
                single_step()
        elif(tokens[0] == "stop"):
            pass
        else:
            rospy.loginfo("Illegal instruction %s", data.data)
        state = WAITING
    else:
        pass

def mission_engine():

    rospy.init_node('mission_engine', anonymous=True)
    param_name = rospy.search_param('mission_home')
    c.MISSION_HOME = rospy.get_param(param_name)

    rospy.Subscriber('host2ros', String, host2ros_callback)

    rospy.Subscriber('cmd_done', Bool, cmd_done_callback)

    rospy.Subscriber('mission_rewrite', String, rewrite_callback)

    global pub_trajectory
    pub_trajectory = rospy.Publisher('trajectory', Float32, queue_size=10)

    global pub_mission_active
    pub_mission_active = rospy.Publisher('mission_active', Bool, queue_size=10)

    global pub_exec_status
    pub_exec_status = rospy.Publisher('ros2host_exec_status', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    mission_engine()
