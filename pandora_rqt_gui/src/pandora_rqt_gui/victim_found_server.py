#!/usr/bin/env python
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
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

import roslib
import rospy
import pandora_fsm
import actionlib

from pandora_rqt_gui.msg import ValidateVictimGUIAction, ValidateVictimGUIResult


class ValidateVictimActionServer(object):

    result = ValidateVictimGUIResult()
    victim_found = False
    operator_responded = False
    victim_valid = False

    def __init__(self, name):
        self.action_name = name

        self.as_ = actionlib.SimpleActionServer(
            self.action_name,
            ValidateVictimGUIAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.as_.start()

    # Victim action found callback
    def execute_cb(self, goal):

        # store  Info
        self.victim_info = [
            goal.victimFoundx, goal.victimFoundy, 4,
            goal.probability, goal.sensorIDsFound]
        # helper variables
        success = True

        if self.as_.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.action_name)
            self.as_.set_preempted()
            success = False

        if success:

            self.victim_found = True
            self.wait_for_response()
            self.operator_responded = False
            self.result.victimValid = self.victim_valid
            self.as_.set_succeeded(self.result)

    def wait_for_response(self):
        while (not self.operator_responded):
            print "I AM WAITTING "
            rospy.sleep(1)

    def shutdown(self):
        rospy.signal_shutdown("SERVER DOWN")


if __name__ == '__main__':
    rospy.init_node('/gui/validate_victim')
    ValidateVictimActionServer('/gui/validate_victim')
    rospy.spin()
