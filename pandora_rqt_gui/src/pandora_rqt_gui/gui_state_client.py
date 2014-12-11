import rospy

from state_manager.state_client import StateClient
from state_manager_msgs.msg import RobotModeMsg


class GuiStateClient(StateClient):

    def __init__(self):

       super(GuiStateClient,self).__init__()

       self.state_ = RobotModeMsg.MODE_OFF
       self.int_to_state_dict = {
           RobotModeMsg.MODE_OFF: "MODE_OFF",
           RobotModeMsg.MODE_START_AUTONOMOUS: "MODE_START_AUTONOMOUS",
           RobotModeMsg.MODE_EXPLORATION_RESCUE: "MODE_EXPLORATION_RESCUE",
           RobotModeMsg.MODE_IDENTIFICATION: "MODE_IDENTIFICATION",
           RobotModeMsg.MODE_SENSOR_HOLD: "MODE_SENSOR_HOLD",
           RobotModeMsg.MODE_SEMI_AUTONOMOUS: "MODE_SEMI_AUTONOMOUS",
           RobotModeMsg.MODE_TELEOPERATED_LOCOMOTION: "MODE_TELEOPERATED_LOCOMOTION",
           RobotModeMsg.MODE_SENSOR_TEST: "MODE_SENSOR_TEST",
           RobotModeMsg.MODE_EXPLORATION_MAPPING:"MODE_EXPLORATION_MAPPING",
           RobotModeMsg.MODE_TERMINATING: "MODE_TERMINATING"
                                  }


    def start_transition(self, state):
        rospy.loginfo("[%s] Starting Transition to state %i", self._name, state)
        self.state_ = state
        self.transition_complete(state)

    def get_state(self):
        return self.int_to_state_dict[self.state_]


    def shutdown(self):
        rospy.signal_shutdown("SERVER DOWN")
