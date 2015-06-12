# Software License Agreement
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Chamzas Konstantinos"
__maintainer__ = "Chamzas Konstantinos"
__email__ = "chamzask@gmail.com"

import rospy

from state_manager.state_client import StateClient
from state_manager_msgs.msg import RobotModeMsg


class GuiStateClient(StateClient):

    def __init__(self):

        super(GuiStateClient, self).__init__()

        self.state_ = RobotModeMsg.MODE_OFF
        self.int_to_state_dict = {
            RobotModeMsg.MODE_OFF: "OFF",
            RobotModeMsg.MODE_START_AUTONOMOUS: "START_AUTONOMOUS",
            RobotModeMsg.MODE_EXPLORATION_RESCUE: "EXPLORATION_RESCUE",
            RobotModeMsg.MODE_IDENTIFICATION: "IDENTIFICATION",
            RobotModeMsg.MODE_SENSOR_HOLD: "SENSOR_HOLD",
            RobotModeMsg.MODE_SEMI_AUTONOMOUS: "SEMI_AUTONOMOUS",
            RobotModeMsg.MODE_TELEOPERATED_LOCOMOTION: "TELEOPERATION",
            RobotModeMsg.MODE_SENSOR_TEST: "SENSOR_TEST",
            RobotModeMsg.MODE_EXPLORATION_MAPPING: "EXPLORATION_MAPPING",
            RobotModeMsg.MODE_TERMINATING: "TERMINATING"
            }

    def start_transition(self, state):

        rospy.loginfo("[%s] Starting Transition to state %i",
                      self._name, state)
        self.state_ = state
        self.transition_complete(state)

    def get_state(self):
        return self.int_to_state_dict[self.state_]

    def shutdown(self):
        rospy.signal_shutdown("SERVER DOWN")
