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

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from pandora_navigation_msgs.msg import DoExplorationAction, \
    DoExplorationFeedback, DoExplorationResult

class MockNavigation():

    def __init__(self, do_exploration_topic, move_base_topic):

        self.robot_pose_ = PoseStamped()

        self.navigation_succedes = True
        self.reply = False
        self.preempted = 0
        
        self.moved_base = False
        self.target_position = Point()
        self.target_orientation = Quaternion() 
        
        self.entered_exploration = False

        self.do_exploration_as_ = SimpleActionServer(
            do_exploration_topic,
            DoExplorationAction,
            execute_cb = self.do_exploration_cb,
            auto_start = False)
        self.do_exploration_as_.start()

        self.move_base_as_ = SimpleActionServer(
            move_base_topic,
            MoveBaseAction,
            execute_cb = self.move_base_cb,
            auto_start = False)
        self.move_base_as_.start()

    def __del__(self):

        self.do_exploration_as_.__del__()
        self.move_base_as_.__del__()

    def do_exploration_cb(self, goal):
        rospy.loginfo('do_exploration_cb')

        self.entered_exploration = True
        while not self.reply:
            rospy.sleep(0.2)
            #self.robot_pose_.pose.position.x += 0.1
            #self.robot_pose_.pose.position.y += 0.05
            #feedback = DoExplorationFeedback()
            #feedback.base_position.pose.position.x = \
            #    self.robot_pose_.pose.position.x
            #feedback.base_position.pose.position.y = \
            #    self.robot_pose_.pose.position.y
            #self.do_exploration_as_.publish_feedback(feedback)
            if self.do_exploration_as_.is_preempt_requested():
                self.preempted += 1
                self.entered_exploration = False
                self.do_exploration_as_.set_preempted()
                break
        else:
            result = DoExplorationResult()
            self.reply = False
            self.preempted = 0
            self.entered_exploration = False
            if self.navigation_succedes:
                self.do_exploration_as_.set_succeded(result) 
            else:
                self.do_exploration_as_.set_aborted(result)

    def move_base_cb(self, goal):
        rospy.loginfo('move_base_cb')

        self.target_position = goal.target_pose.pose.position
        self.target_orientation = goal.target_pose.pose.orientation
        self.moved_base = True
        while not self.reply:
            rospy.sleep(0.2)
            #self.robot_pose_.pose.position.x += 0.1
            #self.robot_pose_.pose.position.y += 0.05
            #feedback = MoveBaseFeedback()
            #feedback.base_position.pose.position.x = \
            #    self.robot_pose_.pose.position.x
            #feedback.base_position.pose.position.y = \
            #    self.robot_pose_.pose.position.y
            #self.move_base_as_.publish_feedback(feedback)
            if self.move_base_as_.is_preempt_requested():
                self.preempted += 1
                self.moved_base = False
                self.move_base_as_.set_preempted()
                break
            if self.move_base_as_.is_new_goal_available():
                self.preempted += 1
                self.move_base_cb(self.move_base_as_.accept_new_goal())
                break
        else:
            result = MoveBaseResult()
            self.reply = False
            self.preempted = 0
            self.moved_base = False
            self.target_position = Point()
            self.target_orientation = Quaternion() 
            if self.navigation_succedes:
                self.move_base_as_.set_succeeded(result)
            else:
                self.move_base_as_.set_aborted(result)
    
if __name__ == '__main__':

    rospy.sleep(0.5)
    rospy.init_node('MockNavigation', anonymous=True)
    navigation = MockNavigation(
        do_exploration_topic = '/navigation/do_exploration',
        move_base_topic = '/move_base')

