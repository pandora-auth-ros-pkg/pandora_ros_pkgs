#!/usr/bin/env python

'''
Title:  cpu_temperature_Title_monitor.py
Description:  CPU temperature monitor node
Author:  George Kouros
'''

import rospy
import sys
import os
import subprocess
import fileinput
from pandora_sensor_msgs.msg import Temperature

class CpuTemperatureMonitor:
    beeper = ['beep', '-f', '1000', '-r', '1']
    temperature_threshold = 84 
    temperature_buffer = []
    buffer_size = 5
    num_cores = 0
    pub_frequency = 1 # Hz
    pub = rospy.Publisher
    msg = Temperature()

    def __init__(self):
      self.pub = rospy.Publisher('/cpu/temperature', Temperature, queue_size=10)
      self.detectPhysicalCpuCores()
      # read temperature
      while not rospy.is_shutdown():
        self.readTemperatures()
        rospy.sleep(1 / self.pub_frequency)


    def detectPhysicalCpuCores(self):
      # find the number of thermal_zone dirs in /sys/class/thermal
      for directory in os.listdir('/sys/class/thermal'):
        if directory.startswith('thermal_zone'):
          self.num_cores += 1
      #initialize temperature buffer
      self.temperature_buffer = [[0] * self.buffer_size for i in range(self.num_cores)]
      # initialize msg
      for core in range(self.num_cores):
        self.msg.name.append('cpu_core_' + str(core));
        self.msg.temperature.append(0)

    def readTemperatures(self):
      # for each core
      for core in range(self.num_cores):
        # remove oldest temperature in buffer of each core
        self.temperature_buffer[core].pop(0)
        # store temperature file path in filename
        filename = '/sys/class/thermal/thermal_zone' + str(core) + '/temp'
        # append new temperature at end of buffer of each core
        self.temperature_buffer[core].append(int(fileinput.input(filename)[0]))
        fileinput.close()
        # calculate average of temperatures in the buffer of the core
        avg_temp = sum(self.temperature_buffer[core]) / self.buffer_size
        # store average temperature of core in msg
        self.msg.temperature[core] = avg_temp / 1000

      # publish the message
      self.pub.publish(self.msg)

      # if (high temperature) enable beeper
      for (name,temp) in zip(self.msg.name, self.msg.temperature):
        if temp > self.temperature_threshold:
          rospy.logerr('Warning: ' + name + ' at : ' + str(temp) + u'\xb0C')
          subprocess.Popen(self.beeper)


if __name__ == '__main__':
  rospy.init_node('cpu_temperature_monitor')
  cpuTemperatureMonitor = CpuTemperatureMonitor()

