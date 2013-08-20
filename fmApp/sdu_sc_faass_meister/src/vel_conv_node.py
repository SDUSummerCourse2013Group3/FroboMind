#!/usr/bin/env python


import rospy
from usb4_driver.msg import analog_io, digital_io
from geometry_msgs.msg import TwistStamped,Twist
from std_srvs.srv import Empty,EmptyResponse
from std_msgs.msg import Bool
class ConversionNode():
    def __init__(self):
        
        self.throttle_pub = rospy.Publisher("/usb4_dac0",analog_io)
        self.cal_pub = rospy.Publisher("/usb4_do0",digital_io)
        self.vel_sub = rospy.Subscriber("/fmSignals/cmd_vel_linear",Twist,self.on_vel)
        self.deadman_sub = rospy.Subscriber("/fmSignals/deadman",Bool,self.deadman)
        self.calibrate = False
        self.cal_msg = digital_io()
        self.cal_msg.Value = 0
        
        self.l_time = rospy.Time.now()
        self.active = False
        
        self.cur_vel = 0.0
        self.throttle_msg = analog_io()
        self.throttle_msg.Value = 2.4
        self.timer = rospy.Timer(rospy.Duration(0.1),self.on_timer)
        self.cur_time = rospy.Time.now()
        self.service = rospy.Service('calibrate_throttle', Empty, self.on_service_req)
        
    def deadman(self,msg):
        if msg.data:
            self.active = True
        else:
            self.active = False
            
    def on_vel(self,msg):
        self.cur_vel = msg.linear.x
        
    def on_timer(self,te):
        
        if (self.cur_time - rospy.Time.now()) > rospy.Duration(0.5):
            self.cur_vel = 0.0
        if self.active == False:
            self.cur_vel = 0.0
        
        self.throttle_msg.Value = self.cur_vel * 0.1 + 2.4
        self.throttle_msg.header.stamp = rospy.Time.now()
        self.throttle_pub.publish(self.throttle_msg)
        
        if self.calibrate:
            self.calibrate = False
            self.cal_msg.Value = 1
            self.cal_msg.header.stamp = rospy.Time.now()
            self.cal_pub.publish(self.cal_msg)
            self.l_time = rospy.Time.now()
        elif (rospy.Time.now() - self.l_time) > rospy.Duration(0.5):
            self.l_time = rospy.Time.now()
            self.cal_msg.Value = 0
            self.cal_msg.header.stamp = rospy.Time.now()
            self.cal_pub.publish(self.cal_msg)
            
    def on_service_req(self,req):
        self.calibrate = True;
        return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node("vel_conversion")
    
    controller = ConversionNode()
    
    rospy.spin()
    
