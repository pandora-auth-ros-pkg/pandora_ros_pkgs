import roslib; roslib.load_manifest('pandora_testing_tools')
import rospy

class realSubscriber:
  
  def __init__(self,topicName,messageType,mockSub):
    rospy.Subscriber(topicName, messageType, self.callback)
    self.mockSub = mockSub
  
  def callback(self,data):
    #~ rospy.loginfo('got data %s', data)
    self.mockSub.callMethod(data)
    
    
class mockSubscriber:
  
  def callMethod(self,data):
    pass
