#!/usr/bin/env python

import rospy
from cs5752_hw1.srv import move_robot
from cs5752_hw1.msg import state

blocks = []

OPEN = False;
CLOSED = True;

TABLE = 0;
NOTHING = -1;

ACTION_OPEN_GRIPPER = 0;
ACTION_CLOSE_GRIPPER = 1;
ACTION_MOVE_TO_BLOCK = 2;
ACTION_MOVE_OVER_BLOCK = 3;

class Block:
    def __init__(self, block_above, block_below):
        self.block_above = block_above
        self.block_below = block_below

class Arm:
    def __init__(self):
        self.gripper_state = OPEN
        self.grasped_block = NOTHING
        self.position_at_block = NOTHING
        self.position_above_block = TABLE

    def open_gripper(self):
        if(self.gripper_state == OPEN):
            return False
        if(self.grasped_block != NOTHING):
            if(self.position_above_block != TABLE):
                blocks[self.position_above_block].block_above = self.grasped_block
            self.gripper_state = OPEN
            self.position_at_block = self.grasped_block
            self.grasped_block = NOTHING
        else:
            self.gripper_state = OPEN
        return True

    def close_gripper(self):
        if(self.gripper_state == CLOSED):
            return False
        self.gripper_state = CLOSED
        self.grasped_block = self.position_at_block
        return True

    def move_to_block(self, block_num):
        if( (block_num <= TABLE) or 
            (block_num >= len(blocks)) or
            (blocks[block_num].block_above != NOTHING) or
            (self.gripper_state == CLOSED) ):
            return False
        self.position_at_block = block_num
        self.position_above_block = blocks[block_num].block_below
        return True

    def move_over_block(self, block_num): 
        if( (block_num == NOTHING) or 
            (block_num == self.grasped_block) or
            (block_num >= len(blocks)) or
            (blocks[block_num].block_above != NOTHING and block_num != TABLE) ):
            return False
        
        if(self.grasped_block!=NOTHING):
            blocks[blocks[self.grasped_block].block_below].block_above = NOTHING
            blocks[self.grasped_block].block_below = block_num
        blocks[block_num].block_above = self.grasped_block
        self.position_at_block = self.grasped_block
        self.position_above_block = block_num
        return True;

arm = Arm()

def command_handler(command):
    if(command.action==ACTION_OPEN_GRIPPER):
        return arm.open_gripper()
    elif(command.action==ACTION_CLOSE_GRIPPER):
        return arm.close_gripper()
    elif(command.action==ACTION_MOVE_TO_BLOCK):
        return arm.move_to_block(command.target)
    elif(command.action==ACTION_MOVE_OVER_BLOCK):
        return arm.move_over_block(command.target)
    else:
        return False
        
    
def sim_master():
    num_blocks = rospy.get_param("num_blocks")
    configuration = rospy.get_param("configuration")

    # append the table
    blocks.append(Block(-1, -1));

    if(num_blocks <= 0):
        raise ValueError("num_blocks must be positive and non-zero")

    if(configuration == 'stacked_ascending'):
        for i in range(0, num_blocks-1):
            blocks.append(Block(i+2,i))
        blocks.append(Block(NOTHING,num_blocks-1))

    elif(configuration == 'stacked_descending'):
        if(num_blocks >1):
            blocks.append(Block(NOTHING,2))  
            for i in range(0, num_blocks-2):
                blocks.append(Block(i+1, i+3))
            blocks.append(Block(num_blocks-1,TABLE))
        else:
            blocks.append(Block(NOTHING,TABLE))

    elif(configuration == 'scattered'):
        for i in range(0, num_blocks):
            blocks.append(Block(NOTHING,TABLE))

    else:
        raise ValueError("configuration must be one of the following: stacked_ascending, stacked_descending or scattered")

    rospy.init_node('sim_master', anonymous=True)
    s = rospy.Service('move_robot', move_robot, command_handler)

    pub = rospy.Publisher('state', state, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        block_is_on = []
        for i in range(1, num_blocks+1):
            block_is_on.append(blocks[i].block_below)
        currentstate = state()
        currentstate.block_is_on = block_is_on
        currentstate.gripper_state = "open" if arm.gripper_state==OPEN else "close"
        currentstate.grasped_block = arm.grasped_block

        pub.publish(currentstate)
        rate.sleep()

if __name__ == '__main__':
    sim_master()