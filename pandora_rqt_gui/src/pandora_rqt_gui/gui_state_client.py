import rospy

from state_manager.state_client import StateClient
from state_manager_communications.msg import robotModeMsg



class GuiStateClient(StateClient):
  


    def __init__(self):

       super(GuiStateClient,self).__init__()

       self._state = robotModeMsg.MODE_OFF
       self._int_to_state_dict = {robotModeMsg.MODE_OFF: "MODE_OFF",
                                  robotModeMsg.MODE_START_AUTONOMOUS: "MODE_START_AUTONOMOUS",
                                  robotModeMsg.MODE_EXPLORATION: "MODE_EXPLORATION",
                                  robotModeMsg.MODE_IDENTIFICATION: "MODE_IDENTIFICATION",
                                  robotModeMsg.MODE_ARM_APPROACH: "MODE_ARM_APPROACH",
                                  robotModeMsg.MODE_DF_HOLD: "MODE_DF_HOLD",
                                  robotModeMsg.MODE_SEMI_AUTONOMOUS: "MODE_SEMI_AUTONOMOUS", 
                                  robotModeMsg.MODE_TELEOPERATED_LOCOMOTION: "MODE_TELEOPERATED_LOCOMOTION",
                                  robotModeMsg.MODE_ARM_TELEOPERATION: "MODE_ARM_TELEOPERATION",
                                  robotModeMsg.MODE_ARM_TELEOPERATION_TUCK: "MODE_ARM_TELEOPERATION_TUCK",
                                  robotModeMsg.MODE_TERMINATING: "MODE_TERMINATING"
                                  }


    def start_transition(self, state):
        rospy.loginfo("[%s] Starting Transition to state %i", self._name, state)
        self._state = state
        self.transition_complete(state)
    
    def getState(self):
        return self._int_to_state_dict[self._state]
        
        
    def shutdown(self):
        rospy.signal_shutdown("SERVER DOWN")
