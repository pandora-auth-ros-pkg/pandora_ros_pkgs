import sys
import rospy
from pandora_geotiff.srv import *
from std_msgs.msg import String

class SaveMissionClient():
    def init():
        pass

    def save_mission_client(self,mission_name):
        print "WAITING FOR SEVICE"
        rospy.wait_for_service('pandora_geotiff_node/saveMission')
        msg = String(mission_name)
        try:
           save_mission = rospy.ServiceProxy('pandora_geotiff_node/saveMission',SaveMission)
           resp = save_mission(msg)
           return True
        except rospy.ServiceException, e:
          print "Service call failed: %s"%e
          
        print "Service call did not fail"
   
if __name__ == "__main__":
  
    print "Requesting save Mission Named Eleana"
