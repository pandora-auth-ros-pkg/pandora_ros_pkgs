#!/usr/bin/env python
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2014, P.A.N.D.O.R.A. Team. All rights reserved."
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
__author__ = "Tsirigotis Christos and Voulgarakis George"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

import rospy
from actionlib import SimpleActionServer

from pandora_rqt_gui.msg import ValidateVictimGUIAction, ValidateVictimGUIResult

class MockGui():

    def __init__(self, gui_validation_topic):

        self.reply = False
        self.preempted = 0
        
        self.victimValid = True

        self.victimFoundx = 0
        self.victimFoundy = 0
        self.probability = 0
        self.sensorIDsFound = []
        
        self.gui_validate_victim_as_ = SimpleActionServer(
            gui_validation_topic, 
            ValidateVictimGUIAction,
            execute_cb = self.gui_validate_victim_cb, 
            auto_start = False)
        self.gui_validate_victim_as_.start()

    def __del__(self):

        self.gui_validate_victim_as_.__del__()

    def gui_validate_victim_cb(self, goal):
        rospy.loginfo('gui_validate_victim_cb')

        self.reply = False
        
        self.victimFoundx = goal.victimFoundx
        self.victimFoundy = goal.victimFoundy
        self.probability = goal.probability
        self.sensorIDsFound = goal.sensorIDsFound

        while not self.reply:
            rospy.sleep(0.5)

            if self.gui_validate_victim_as_.is_preempt_requested():
                preempted += 1
                self.gui_validate_victim_as_.set_preempted()
                break
        else:
            self.preempted = 0
            result = ValidateVictimGUIResult(victimValid = self.victimValid) 
            self.gui_validate_victim_as_.set_succeeded(result)

if __name__ == '__main__':

    rospy.sleep(0.5)
    rospy.init_node('MockGui', anonymous=True)
    gui = MockGui(gui_validation_topic = '/gui/validate_victim')
