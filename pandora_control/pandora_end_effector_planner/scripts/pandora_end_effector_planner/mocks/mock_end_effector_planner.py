#!/usr/bin/env python
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
__author__ = "Voulgarakis George and Tsirigotis Christos"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

import rospy
from actionlib import SimpleActionServer

from pandora_end_effector_planner.msg import MoveEndEffectorAction, \
    MoveEndEffectorResult, MoveEndEffectorGoal

class MockEndEffectorPlanner():

    def __init__(self, end_effector_planner_topic):

        self.moves_end_effector = False
        self.move_end_effector_succeeded = False
        self.reply = False

        self.command = 0
        self.point_of_interest = ""
        self.center_point = ""
        
        self.end_effector_planner_as_ = SimpleActionServer(
            end_effector_planner_topic,
            MoveEndEffectorAction,
            execute_cb=self.end_effector_planner_cb,
            auto_start=False)
        self.end_effector_planner_as_.start()

    def __del__(self):

        self.end_effector_planner_as_.__del__()

    def end_effector_planner_cb(self, goal):
        rospy.loginfo('end_effector_planner_cb')

        self.moves_end_effector = True
        self.command = goal.command
        self.point_of_interest = goal.point_of_interest
        self.center_point = goal.center_point

        while not self.reply:
            rospy.sleep(1.)
            if self.end_effector_planner_as_.is_preempt_requested():
                self.end_effector_planner_as_.set_preempted(MoveEndEffectorResult())
                self.moves_end_effector = False
                return None
        else:
            self.reply = False
            result = MoveEndEffectorResult()
            if self.move_end_effector_succeeded:
                self.moves_end_effector = False
                self.end_effector_planner_as_.set_succeeded(result)
            else:
                self.moves_end_effector = False
                self.end_effector_planner_as_.set_aborted(result)

