#!/usr/bin/env python

''' 
File Description    State indicator node.
Author            Chris Zalidis
'''

import rospy
import serial
import sys
from pandora_xmega_hardware_interface.msg import BatteryMsg
from state_manager_communications.msg import robotModeMsg


class StateIndicator:
    
    def __init__(self):
        rospy.Subscriber('/sensors/battery', BatteryMsg, self.butterfly_callback)
        rospy.Subscriber('/robot/state/clients', robotModeMsg, self.state_callback)
        self.ser = serial.Serial("/dev/state_indicator", 38400)

        
    def butterfly_callback(self, msg):
        
        self.ser.write("T21.0"+"M"+str(max(msg.voltage[1],0.0))+"P"+str(max(msg.voltage[0],0.0)))
        
    def state_callback(self, msg):
        
        if msg.type == msg.TYPE_TRANSITION:
            if msg.mode < 10:
                self.ser.write("S0"+str(msg.mode))
            else:
                self.ser.write("S"+str(msg.mode))


if __name__ == '__main__':
    rospy.init_node('state_indicator')
    monitor = StateIndicator()
    rospy.spin()
