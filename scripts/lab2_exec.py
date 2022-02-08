#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([162.33, -109.69, 115.25, -94.18, -89.5, 129.27])

# Hanoi tower location 0
Q10 = [156.27*pi/180.0, -64.18*pi/180.0, 73.39*pi/180.0, -97.79*pi/180.0, -89.97*pi/180.0, 123.18*pi/180.0]
Q20 = [170.42*pi/180.0, -64.92*pi/180.0, 78.04*pi/180.0, -101.81*pi/180.0, -89.64*pi/180.0, 137.32*pi/180.0]
Q30 = [181.94*pi/180.0, -59.20*pi/180.0, 69.67*pi/180.0, -99.31*pi/180.0, -89.42*pi/180.0, 148.83*pi/180.0]


# Hanoi tower location 1
Q11 = [155.93*pi/180.0, -53.72*pi/180.0, 90.45*pi/180.0, -125.32*pi/180.0, -90.03*pi/180.0, 122.77*pi/180.0]
Q12 = [155.94*pi/180.0, -48.81*pi/180.0, 92.35*pi/180.0, -132.11*pi/180.0, -90.05*pi/180.0, 122.74*pi/180.0]
Q13 = [155.94*pi/180.0, -43.78*pi/180.0, 93.08*pi/180.0, -137.88*pi/180.0, -90.07*pi/180.0, 122.73*pi/180.0]

# Hanoi tower location 2
Q21 = [170.36*pi/180.0, -54.93*pi/180.0, 91.90*pi/180.0, -125.65*pi/180.0, -89.68*pi/180.0, 137.2*pi/180.0]
Q22 = [170.37*pi/180.0, -49.93*pi/180.0, 93.90*pi/180.0, -132.66*pi/180.0, -89.69*pi/180.0, 137.19*pi/180.0]
Q23 = [170.38*pi/180.0, -44.67*pi/180.0, 94.70*pi/180.0, -138.71*pi/180.0, -89.72*pi/180.0, 137.17*pi/180.0]

# Hanoi tower location 3
Q31 = [182.11*pi/180.0, -50.72*pi/180.0, 84.29*pi/180.0, -122.4*pi/180.0, -89.46*pi/180.0, 148.83*pi/180.0]
Q32 = [182.12*pi/180.0, -45.41*pi/180.0, 86.45*pi/180.0, -129.88*pi/180.0, -89.47*pi/180.0, 148.91*pi/180.0]
Q33 = [182.13*pi/180.0, -41.34*pi/180.0, 87.05*pi/180.0, -134.55*pi/180.0, -89.49*pi/180.0, 148.89*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q10, Q11, Q12, Q13], \
      [Q20, Q21, Q22, Q23], \
      [Q30, Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""




############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".

    # Go to height right above start height at start pos 
    if(start_loc == 0):
        move_arm(pub_cmd, loop_rate, Q10, 4.0, 4.0)
    elif(start_loc == 1):
        move_arm(pub_cmd, loop_rate, Q20, 4.0, 4.0)
    elif(start_loc == 2):
        move_arm(pub_cmd, loop_rate, Q30, 4.0, 4.0)

    # Turn gripper on 
    gripper(pub_cmd, loop_rate, suction_on)

    time.sleep(1.0)

    # Move down to height of block 
    if(start_loc == 0):
        if(start_height == 1):
            move_arm(pub_cmd, loop_rate, Q11, 4.0, 4.0)
        elif(start_height == 2):
            move_arm(pub_cmd, loop_rate, Q12, 4.0, 4.0)
        elif(start_height == 3):
            move_arm(pub_cmd, loop_rate, Q13, 4.0, 4.0)
    elif(start_loc == 1):
        if(start_height == 1):
            move_arm(pub_cmd, loop_rate, Q21, 4.0, 4.0)
        elif(start_height == 2):
            move_arm(pub_cmd, loop_rate, Q22, 4.0, 4.0)
        elif(start_height == 3):
            move_arm(pub_cmd, loop_rate, Q23, 4.0, 4.0)
    elif(start_loc == 2):
        if(start_height == 1):
            move_arm(pub_cmd, loop_rate, Q31, 4.0, 4.0)
        elif(start_height == 2):
            move_arm(pub_cmd, loop_rate, Q32, 4.0, 4.0)
        elif(start_height == 3):
            move_arm(pub_cmd, loop_rate, Q33, 4.0, 4.0)

    # Wait 1 sec 
    time.sleep(1.0)

    # Go to height right above start height at start pos 
    if(start_loc == 0):
        move_arm(pub_cmd, loop_rate, Q10, 4.0, 4.0)
    elif(start_loc == 1):
        move_arm(pub_cmd, loop_rate, Q20, 4.0, 4.0)
    elif(start_loc == 2):
        move_arm(pub_cmd, loop_rate, Q30, 4.0, 4.0)

    # Go to height right above end height at end pos
    if(end_loc == 0):
        move_arm(pub_cmd, loop_rate, Q10, 4.0, 4.0)
    elif(end_loc == 1):
        move_arm(pub_cmd, loop_rate, Q20, 4.0, 4.0)
    elif(end_loc == 2):
        move_arm(pub_cmd, loop_rate, Q30, 4.0, 4.0)
    
    # Wait 1 sec
    time.sleep(1.0)
    
    # Move down to end height
    if(end_loc == 0):
        if(end_height == 1):
            move_arm(pub_cmd, loop_rate, Q11, 4.0, 4.0)
        elif(end_height == 2):
            move_arm(pub_cmd, loop_rate, Q12, 4.0, 4.0)
        elif(end_height == 3):
            move_arm(pub_cmd, loop_rate, Q13, 4.0, 4.0)
    elif(end_loc == 1):
        if(end_height == 1):
            move_arm(pub_cmd, loop_rate, Q21, 4.0, 4.0)
        elif(end_height == 2):
            move_arm(pub_cmd, loop_rate, Q22, 4.0, 4.0)
        elif(end_height == 3):
            move_arm(pub_cmd, loop_rate, Q23, 4.0, 4.0)
    elif(end_loc == 2):
        if(end_height == 1):
            move_arm(pub_cmd, loop_rate, Q31, 4.0, 4.0)
        elif(end_height == 2):
            move_arm(pub_cmd, loop_rate, Q32, 4.0, 4.0)
        elif(end_height == 3):
            move_arm(pub_cmd, loop_rate, Q33, 4.0, 4.0)

    # Wait 1 sec
    time.sleep(1.0)

     #Turn off gripper
    gripper(pub_cmd, loop_rate, suction_off)

    # Go to height right above end height at end pos
    if(end_loc == 0):
        move_arm(pub_cmd, loop_rate, Q10, 4.0, 4.0)
    elif(end_loc == 1):
        move_arm(pub_cmd, loop_rate, Q20, 4.0, 4.0)
    elif(end_loc == 2):
        move_arm(pub_cmd, loop_rate, Q30, 4.0, 4.0)

    # end 
    error = 0
    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function


    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    # input_done = 0
    # loop_count = 0

    # while(not input_done):
    #     input_string = raw_input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
    #     print("You entered " + input_string + "\n")

    #     if(int(input_string) == 1):
    #         input_done = 1
    #         loop_count = 1
    #     elif (int(input_string) == 2):
    #         input_done = 1
    #         loop_count = 2
    #     elif (int(input_string) == 3):
    #         input_done = 1
    #         loop_count = 3
    #     elif (int(input_string) == 0):
    #         print("Quitting... ")
    #         sys.exit()
    #     else:
    #         print("Please just enter the character 1 2 3 or 0 to quit \n\n")


    # Store starting position for tower
    start_pos = 0
    # Store end position for tower
    end_pos = 0

    not_done = True

    while(not_done):

        input_string = raw_input("What would you like your start position to be? Please choose 1, 2, or 3. Press 0 to quit. ")
        print("You entered " + input_string + "\n")
        # Verify input and store it in start_pos
        if(int(input_string) == 1):
            start_pos = 1 
        elif(int(input_string) == 2):
            start_pos = 2
        elif(int(input_string) == 3):
            start_pos = 3
        elif(int(input_string) == 0):
            print("Quitting...")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")
        
        input_string = raw_input("What would you like your end position to be? Please choose 1, 2, or 3. Press 0 to quit. ")
        print("You entered " + input_string + "\n")
        # Verify input and store it in end_pos
        if(int(input_string) == 1):
            end_pos = 1 
            not_done = False
        elif(int(input_string) == 2):
            end_pos = 2
            not_done = False
        elif(int(input_string) == 3):
            end_pos = 3
            not_done = False
        elif(int(input_string) == 0):
            print("Quitting...")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    # while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)

    # Move arm to home position
    move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    time.sleep(1.0)

    # Rule for start_pos = 1 and end_pos = 3
    if(start_pos == 1 and end_pos == 3):
        # # Move to 1 pos 4 block height
        # rospy.loginfo("Sending goal 1 ...")
        # move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)
        # # Grip block
        # gripper(pub_command, loop_rate, suction_on)
        # # Move to 1 pos 3 block height\
        move_block(pub_command, loop_rate, 0, 1, 2, 3)
        move_block(pub_command, loop_rate, 0, 2, 1, 3)
        move_block(pub_command, loop_rate, 2, 3, 1, 2)
        move_block(pub_command, loop_rate, 0, 3, 2, 3)
        move_block(pub_command, loop_rate, 1, 2, 0, 3)
        move_block(pub_command, loop_rate, 1, 3, 2, 2)
        move_arm(pub_command, loop_rate, Q30, 4.0, 4.0)
        move_block(pub_command, loop_rate, 0, 3, 2, 1)
        move_arm(pub_command, loop_rate, Q30, 4.0, 4.0)
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    if(start_pos == 3 and end_pos == 1):
        move_block(pub_command, loop_rate, 2, 1, 0, 3)
        move_block(pub_command, loop_rate, 2, 2, 1, 3)
        move_block(pub_command, loop_rate, 0, 3, 1, 2)
        move_block(pub_command, loop_rate, 2, 3, 0, 3)
        move_block(pub_command, loop_rate, 1, 2, 2, 3)
        move_block(pub_command, loop_rate, 1, 3, 0, 2)
        move_arm(pub_command, loop_rate, Q10, 4.0, 4.0)
        move_block(pub_command, loop_rate, 2, 3, 0, 1)
        move_arm(pub_command, loop_rate, Q10, 4.0, 4.0)
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    if(start_pos == 2 and end_pos == 3):
        move_block(pub_command, loop_rate, 1, 1, 2, 3)
        move_block(pub_command, loop_rate, 1, 2, 0, 3)
        move_block(pub_command, loop_rate, 2, 3, 0, 2)
        move_block(pub_command, loop_rate, 1, 3, 2, 3)
        move_block(pub_command, loop_rate, 0, 2, 1, 3)
        move_block(pub_command, loop_rate, 0, 3, 2, 2)
        move_arm(pub_command, loop_rate, Q30, 4.0, 4.0)
        move_block(pub_command, loop_rate, 1, 3, 2, 1)
        move_arm(pub_command, loop_rate, Q30, 4.0, 4.0)
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    if(start_pos == 3 and end_pos == 2):
        move_block(pub_command, loop_rate, 2, 1, 1, 3)
        move_block(pub_command, loop_rate, 2, 2, 0, 3)
        move_block(pub_command, loop_rate, 1, 3, 0, 2)
        move_block(pub_command, loop_rate, 2, 3, 1, 3)
        move_block(pub_command, loop_rate, 0, 2, 2, 3)
        move_arm(pub_command, loop_rate, Q10, 4.0, 4.0)
        move_block(pub_command, loop_rate, 0, 3, 1, 2)
        move_arm(pub_command, loop_rate, Q20, 4.0, 4.0)
        move_block(pub_command, loop_rate, 2, 3, 1, 1)
        move_arm(pub_command, loop_rate, Q20, 4.0, 4.0)
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    if(start_pos == 1 and end_pos == 2):
        move_block(pub_command, loop_rate, 0, 1, 1, 3)
        move_block(pub_command, loop_rate, 0, 2, 2, 3)
        move_block(pub_command, loop_rate, 1, 3, 2, 2)
        move_block(pub_command, loop_rate, 0, 3, 1, 3)
        move_block(pub_command, loop_rate, 2, 2, 0, 3)
        move_arm(pub_command, loop_rate, Q30, 4.0, 4.0)
        move_block(pub_command, loop_rate, 2, 3, 1, 2)
        move_arm(pub_command, loop_rate, Q20, 4.0, 4.0)
        move_block(pub_command, loop_rate, 0, 3, 1, 1)
        move_arm(pub_command, loop_rate, Q20, 4.0, 4.0)
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    if(start_pos == 2 and end_pos == 1):
        move_block(pub_command, loop_rate, 1, 1, 0, 3)
        move_block(pub_command, loop_rate, 1, 2, 2, 3)
        move_block(pub_command, loop_rate, 0, 3, 2, 2)
        move_block(pub_command, loop_rate, 1, 3, 0, 3)
        move_block(pub_command, loop_rate, 2, 2, 1, 3)
        move_arm(pub_command, loop_rate, Q20, 4.0, 4.0)
        move_block(pub_command, loop_rate, 2, 3, 0, 2)
        move_arm(pub_command, loop_rate, Q20, 4.0, 4.0)
        move_block(pub_command, loop_rate, 1, 3, 0, 1)
        move_arm(pub_command, loop_rate, Q20, 4.0, 4.0)
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)


    # gripper(pub_command, loop_rate, suction_on)
    # # Delay to make sure suction cup has grasped the block
    # time.sleep(1.0)

    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
