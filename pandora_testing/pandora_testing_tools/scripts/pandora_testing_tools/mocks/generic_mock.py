#!/usr/bin/env python

PKG = 'pandora_testing_tools'
import roslib; 
import rospy
import actionlib
import unittest
import mox
from time import sleep
import rosbag
import sys
import mocksubscriber
import moxcomparators

from pandora_testing_tool.msg import *

class SubscriberMockUnitTest(unittest.TestCase):

  def setUp(self):
    
    self.topics2MockObjects = {}
    self.mocker = mox.Mox()

    self.bagfile = rospy.get_param("/bag_filename")

    #~ construct a dictionary holding for each topic requested, a mock object 
    #~ subscribing to it
    topics = rospy.get_param("/topic_names")
    for topic, messagePackage, messageType in topics:
      _temp = __import__(messagePackage+'.msg', globals(), locals(), messageType, -1)
      messageTypeObj = getattr(_temp, messageType)
      mockObj = self.mocker.CreateMock(mocksubscriber.mockSubscriber)
      mocksubscriber.realSubscriber(topic, messageTypeObj, mockObj)
      self.topics2MockObjects[topic] = mockObj
    
    
  def testUsingMox(self):
    
    #~ open a bag and register all the messages to the 
    #~ mock object of the corresponding topic (if the topic is listed in the given)
    for topic, msg, t in rosbag.Bag(self.bagfile).read_messages(): 
      if topic in self.topics2MockObjects :
        self.topics2MockObjects[topic].callMethod(moxcomparators.msgEquals(msg))
    
    #~ Tell the mock objects that we finished registering method calls and are now
    #~ verifying
    self.mocker.ReplayAll()
    
    #~ Call the action that replays the bag. Here we use the same bag to check if everything works
    #~ In real testing this should be replaced by waiting for the real code to produce output
    #~ Nevertheless, the bag replay action could still be used for something else
    #~ (maybe change to service??)
    client = actionlib.SimpleActionClient('/test/bag_player', ReplayBagsAction)
    client.wait_for_server()

    goal = ReplayBagsGoal()
    goal.start = True
    client.send_goal(goal)
    client.wait_for_result()
    
    #~ Verify that the registered callbacks are called , i.e. all the messages in the 
    #~ bag we opened were heard in this order
    self.mocker.VerifyAll()
    

if __name__ == '__main__':
  
    import rostest
    
    argv = rospy.myargv(argv=sys.argv)
  
    rospy.init_node('tester')
    
    rostest.rosrun(PKG, 'test_using_mox', SubscriberMockUnitTest) 

  
