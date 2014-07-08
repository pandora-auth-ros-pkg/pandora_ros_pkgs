#!/usr/bin/env python

''' 
File Description    Battery monitor node. Beeps when battery under 20.5 Volts.
Author            Chris Zalidis
'''

import rospy
import subprocess
import sys
from pandora_xmega_hardware_interface.msg import BatteryMsg


class BatteryMonitor:

    beeper = ['beep','-f','3000','-r','20']
    highest_voltage = []
    previous_voltage = []
    ok = []
    init = False
    
    def __init__(self):
        rospy.Subscriber('/sensors/battery', BatteryMsg, self.callback)
        
    def callback(self, data):

        if not self.init:
            self.highest_voltage = len(data.name) * [0]
            self.previous_voltage = len(data.name) * [0]
            self.ok = len(data.name) * [True]
            self.init = True

        for i in range(len(data.name)):

            if data.voltage[i] > 20.7 and self.previous_voltage[i] < 20.5:
                self.ok[i] = True

            self.highest_voltage[i] = max(data.voltage[i], self.highest_voltage[i])

            if self.highest_voltage[i] > 20 and data.voltage[i] < 20.5:
                rospy.logerr(str(data.name[i]) + ' '+ str(data.voltage[i]) + 'V !!')
                if self.ok[i]:
                    subprocess.Popen(self.beeper)
                self.ok[i] = False

            self.previous_voltage[i] = data.voltage[i]


if __name__ == '__main__':
    rospy.init_node('battery_monitor')
    monitor = BatteryMonitor()
    rospy.spin()
