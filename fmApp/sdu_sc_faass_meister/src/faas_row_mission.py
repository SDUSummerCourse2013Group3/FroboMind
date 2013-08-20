#!/usr/bin/env python

# Import generic python libraries
import threading
import rospy

import smach
import smach_ros
# Actions used in this statemachine
from action_primitives.msg import navigate_rowGoal

from wii_interface import wii_interface 
from turn_behaviours.u_turn import build_u_turn_sm
from row_behaviours.navigate_row_behaviour import build_row_nav_sm
from generic_smach.states import wii_states
from std_srvs.srv import Empty
    

class FaasRowMission():
    def __init__(self):
        self.sm = None
        self.intro_server = None
        self.smach_thread = None
        self.hmi = None
        pass
        
        
    def build(self):
        
        auto = self.build_row_nav()
        
        autonomous = smach.Concurrence(  outcomes = ['exitAutomode','aborted'],
                                                default_outcome = 'exitAutomode',
                                                outcome_map = {'exitAutomode':{'HMI':'preempted','ROWNAV':'preempted'},
                                                               'aborted':{'HMI':'preempted','ROWNAV':'aborted'}},
                                                child_termination_cb = lambda ud: True )
        with autonomous:
            smach.Concurrence.add('HMI', wii_states.interfaceState(self.hmi))
            smach.Concurrence.add('ROWNAV', auto)
        
        # Build the top level mission control from the remote control state and the autonomous state
        mission_control = smach.StateMachine(outcomes=['preempted','aborted'])            
        with mission_control:
            smach.StateMachine.add('REMOTE_CONTROL', wii_states.remoteControlState(self.hmi), transitions={'enterAutomode':'AUTO_MODE','preempted':'preempted'})
            smach.StateMachine.add('AUTO_MODE', autonomous, transitions={'exitAutomode':'REMOTE_CONTROL','aborted':'REMOTE_CONTROL'})
        return mission_control
    
    def build_row_nav(self):
        row_goal = navigate_rowGoal()
        row_goal.desired_offset_from_row = 0
        row_goal.distance_scale = -0.35
        row_goal.forward_velcoity = 0.8
        row_goal.headland_timeout = 5
        row_goal.P = 0.3
        
        row_nav = build_row_nav_sm(row_goal,2)
    
        #length_in, length_out, width, turn_radius , direction_l,vel_fw,vel_turn, fix_offset):
        uturn_right = build_u_turn_sm(7,3, 9.5, 3, False, 0.4, 0.4,0)
        uturn_left = build_u_turn_sm(7,3, 8.1, 3, True, 0.4, 0.4,0)
        
        main_sm = smach.StateMachine(["succeeded","aborted","preempted"])
        main_sm.userdata.next_turn = "left"
        
        with main_sm:
            smach.StateMachine.add("NAVIGATE_IN_ROW",
                                    row_nav,
                                    transitions={"succeeded":"TURN_SELECTOR"},
                                    )
            smach.StateMachine.add("TURN_SELECTOR",
                                   smach.CBState(self.on_turn_selection, outcomes=["left","right"], input_keys=["next_turn_in"], output_keys=["next_turn_out"]),
                                   transitions={"left":"MAKE_TURN_LEFT","right":"MAKE_TURN_RIGHT"},
                                   remapping = {"next_turn_in":"next_turn","next_turn_out":"next_turn"}
                                   )
            smach.StateMachine.add("MAKE_TURN_RIGHT",
                                   uturn_right,
                                   transitions={"succeeded":"NAVIGATE_IN_ROW"}
                                   )
            smach.StateMachine.add("MAKE_TURN_LEFT",
                                   uturn_left,
                                   transitions={"succeeded":"NAVIGATE_IN_ROW"}
                                   )

        return main_sm
    
    def on_home(self):
        print "calibrating"
        self.calibrate()
        print "done"

    def start(self):
        
        rospy.init_node("field_mission")
        
        self.hmi = wii_interface.WiiInterface()
        self.hmi.register_callback_button_home(self.on_home)
        
        print "waiting for service"
        self.calibrate = rospy.ServiceProxy('/calibrate_throttle', Empty)
        rospy.wait_for_service(self.calibrate)
        print "got service"
        
        try:
            self.sm = self.build()
            self.intro_server = smach_ros.IntrospectionServer('field_mission',self.sm,'/FIELDMISSION')
    
            self.intro_server.start()    
    
            self.smach_thread = threading.Thread(target = self.sm.execute)
            self.smach_thread.start()
        except Exception as e:
            print e
            raise Exception("Could not start ")
        
        
    def stop(self):
        print "stopping"
        if self.sm:
            print "closing state_machine"
            self.sm.request_preempt()
            
        if self.intro_server:
            print "stopping intro server"
            self.intro_server.stop()
            
        if self.smach_thread:
            print "waiting for smach thread to join"
            self.smach_thread.join()
            print "smach thread joined"
            
        
    @smach.cb_interface(input_keys=["next_turn_in"],output_keys=["next_turn_out"],outcomes=['left','right'])
    def on_turn_selection(self,ud):
        if ud.next_turn_in == "left":
            ud.next_turn_out = "right"
            return 'left'
        else:
            ud.next_turn_out = "left"
            return 'right'

    
if __name__ == "__main__":
    
    rospy.init_node("field_mission")
    try:
        nav = FaasRowMission()
        nav.start()
        rospy.spin()
        nav.stop()
    except Exception as e:
        print type(e)
        print e
        print "Caught error"
        rospy.signal_shutdown("could not start")
        nav.stop()
        print "stopped"

