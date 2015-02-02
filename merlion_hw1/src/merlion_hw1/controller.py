#!/usr/bin/env python

import rospy
from cs5752_hw1.srv import move_robot
from std_msgs.msg import String

num_blocks = 0

TABLE = 0;
NOTHING = -1;

ACTION_OPEN_GRIPPER = 0;
ACTION_CLOSE_GRIPPER = 1;
ACTION_MOVE_TO_BLOCK = 2;
ACTION_MOVE_OVER_BLOCK = 3;

def scatter_all_blocks():
    rospy.wait_for_service('move_robot')
    move_robot_p = rospy.ServiceProxy('move_robot', move_robot)
    move_robot_p(ACTION_OPEN_GRIPPER, 0)

    already_scattered = True

    for i in range(1,num_blocks+1) :
        try:
            if (not move_robot_p(ACTION_MOVE_TO_BLOCK, i).success):
                already_scattered = False
                break
        except rospy.ServiceException, e:
            rospy.logerr( "Service call failed: %s"%e)

    move_robot_p(ACTION_OPEN_GRIPPER, 0)
    move_robot_p(ACTION_MOVE_OVER_BLOCK,TABLE)
    if (already_scattered==False):    
        for i in range(1,num_blocks+1):
            try:
                move_robot_p(ACTION_MOVE_TO_BLOCK,i)
                move_robot_p(ACTION_CLOSE_GRIPPER,i)
                move_robot_p(ACTION_MOVE_OVER_BLOCK,TABLE)
                move_robot_p(ACTION_OPEN_GRIPPER,i)         
            except rospy.ServiceException, e:
                rospy.logerr( "Service call failed: %s"%e)
        scatter_all_blocks();


def stack_ascending():
    rospy.wait_for_service('move_robot')
    move_robot_p = rospy.ServiceProxy('move_robot', move_robot)
    if(num_blocks>1):
        for i in range(2, num_blocks+1):
            try:
                move_robot_p(ACTION_MOVE_TO_BLOCK,i)
                move_robot_p(ACTION_CLOSE_GRIPPER,i)
                move_robot_p(ACTION_MOVE_OVER_BLOCK,i-1)
                move_robot_p(ACTION_OPEN_GRIPPER,i)     
            except rospy.ServiceException, e:
                rospy.logerr( "Service call failed: %s"%e)


def stack_descending():
    rospy.wait_for_service('move_robot')
    move_robot_p = rospy.ServiceProxy('move_robot', move_robot)
    if(num_blocks>1):
        for i in reversed(range(1, num_blocks)):
            try:
                move_robot_p(ACTION_MOVE_TO_BLOCK,i)
                move_robot_p(ACTION_CLOSE_GRIPPER,i)
                move_robot_p(ACTION_MOVE_OVER_BLOCK,i+1)
                move_robot_p(ACTION_OPEN_GRIPPER,i)  
            except rospy.ServiceException, e:
                rospy.logerr( "Service call failed: %s"%e)


def perform_command(command):
    if(command.data=='scatter'):
        scatter_all_blocks()
    elif(command.data=='stack_ascending'):
        scatter_all_blocks()
        stack_ascending()
    elif(command.data=='stack_descending'):
        scatter_all_blocks()
        stack_descending()
    else:
        rospy.logerr( "invalid command: %s", command.data)


def controller():
    rospy.init_node('controller', anonymous=True)
    global num_blocks
    num_blocks = rospy.get_param("num_blocks")

    rospy.Subscriber("command", String, perform_command)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    controller()