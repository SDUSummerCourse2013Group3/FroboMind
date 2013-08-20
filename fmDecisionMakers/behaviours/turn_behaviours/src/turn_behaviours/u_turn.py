#!/usr/bin/env python
import rospy

import math

import actionlib

import smach
import smach_ros

# Actions used in this statemachine
from action_primitives.msg import drive_forwardGoal, drive_forwardAction,make_turnGoal,make_turnAction

def build_u_turn_sm(length_in, length_out, width, turn_radius , direction_l,vel_fw,vel_turn,fix_offset):
    """
    Generates a SMACH state machine which will perform a u-turn. 
    @param length_in: The length to drive forward before initiating the first turn
    @param length_out: The length to drive forward after the last turn has been performed
    @param width: the width of the uturn.
    @param turn_radius: the turn radius to use, be careful as this parameter is not checked
    @param direction_l: True if the uturn should be performed to the left
    @param vel_fw: The forward velocity to use when performing the u-turn
    @param vel_turn: The turning velocity to use
    @param fix_offset: A parameter used to compensate for small direction errors in the vehicle (Hakotrac) set to 0
    """
    # vel is in m/s turn radius is in meter
    #if not direction_l:
    #    turn = vel_turn/turn_radius + 0.08
    #else:
    #    turn = vel_turn/turn_radius
    turn = vel_turn/turn_radius
    
    if direction_l:
        lr_amount = 1.42
    else:
        lr_amount = -1.42
     
    uturn_sm = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    with uturn_sm:
        smach.StateMachine.add("DRIVE_FW_IN", 
                               smach_ros.SimpleActionState("/fmExecutors/drive_forward", 
                                                           drive_forwardAction, 
                                                           goal = drive_forwardGoal(amount=(length_in-turn_radius), vel=vel_fw, fix_offset=fix_offset ) ), 
                               transitions={"succeeded":"TURN_1"}, 
                               )
        smach.StateMachine.add("TURN_1",
                               smach_ros.SimpleActionState("/fmExecutors/make_turn",
                                                           make_turnAction,
                                                           goal= make_turnGoal(amount = lr_amount, vel=turn, forward_vel=vel_turn )),
                               transitions={"succeeded":"DRIVE_FW_CROSS"}
                               )
        smach.StateMachine.add("DRIVE_FW_CROSS", 
                               smach_ros.SimpleActionState("/fmExecutors/drive_forward", 
                                                           drive_forwardAction, 
                                                           goal = drive_forwardGoal(amount=(width-turn_radius*2), vel=vel_fw, fix_offset=fix_offset ) ), 
                               transitions={"succeeded":"TURN_2"}, 
                               )
        smach.StateMachine.add("TURN_2",
                               smach_ros.SimpleActionState("/fmExecutors/make_turn",
                                                           make_turnAction,
                                                           goal= make_turnGoal(amount = lr_amount, vel=turn, forward_vel=vel_turn)),
                               transitions={"succeeded":"DRIVE_FW_OUT"}
                               )
        smach.StateMachine.add("DRIVE_FW_OUT",
                               smach_ros.SimpleActionState("/fmExecutors/drive_forward", 
                                                           drive_forwardAction, 
                                                           goal = drive_forwardGoal(amount=(length_out-turn_radius), vel=vel_fw, fix_offset=fix_offset ) ), 
                               transitions={"succeeded":"succeeded"}, 
                               )
    
    return uturn_sm
