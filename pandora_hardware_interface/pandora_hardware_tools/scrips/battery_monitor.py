#!/usr/bin/env python

''' 
File Description	Battery monitor node. Beeps when battery under 21 Volts.
Contents		callback
Author			Chris Zalidis
Date			16/4/2013
Change History		
'''

import roslib; roslib.load_manifest('pandora_hardware_tools')
import rospy
import subprocess
import sys
from controllers_and_sensors_communications.msg import butterflyMsg


class BatteryMonitor:
	
	highestVoltageMotors = 0
	highestVoltagePSU = 0
	previousMotors = 0
	previousPSU = 0
	okMotor = True
	okPSU = True
	
	def __init__(self):
		rospy.Subscriber('/sensors/butterfly', butterflyMsg, self.callback)
		
	def callback(self, data):
		beeper = ['beep','-f','3000','-r','100']
		
		if data.voltage[0] > 20 and self.previousMotors < 20:
			self.okMotor = True
		if data.voltage[1] > 20 and self.previousPSU < 20:
			self.okPSU = True
		
		self.highestVoltageMotors = max(data.voltage[0], self.highestVoltageMotors)
		self.highestVoltagePSU = max(data.voltage[1], self.highestVoltagePSU)
		
		if self.highestVoltageMotors > 20 and data.voltage[0] < 21:
			rospy.logerr('Motor battery '+ str(data.voltage[0]) + 'V !!')
			if self.okMotor:
				subprocess.Popen(beeper)
			self.okMotor = False
			
		if self.highestVoltagePSU > 20 and data.voltage[1] < 21:
			rospy.logerr('PSU battery ' + str(data.voltage[1]) + 'V !!')
			if self.okPSU:
				subprocess.Popen(beeper)
			self.okPSU = False
			
		self.previousMotors = data.voltage[0]
		self.previousPSU = data.voltage[1]


if __name__ == '__main__':
    rospy.init_node('battery_monitor')
    monitor = BatteryMonitor()
    rospy.spin()
