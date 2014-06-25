#!/usr/bin/env python

import roslib
import rospy
import pandora_fsm
import actionlib
from pandora_rqt_gui.msg import ValidateVictimGUIAction, ValidateVictimGUIResult
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QTime, QObject


class ValidateVictimActionServer(object):

    _result = ValidateVictimGUIResult()
    _victimFound = False
    _operatorResponded = False
    _victimValid = False

    def __init__(self, name):
        self._action_name = name

        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ValidateVictimGUIAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self._as.start()

    # Victim action found callback
    def execute_cb(self, goal):

        # store  Info
        self._victimInfo = [goal.victimFoundx, goal.victimFoundy, 4, goal.probability, goal.sensorIDsFound]
        # helper variables
        success = True

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

        if success:

            self._victimFound = True
            self.wait_for_response()
            self._operatorResponded = False
            self._result.victimValid = self._victimValid
            self._as.set_succeeded(self._result)

    def wait_for_response(self):
        while (not self._operatorResponded):
            print "I AM WAITTING "
            rospy.sleep(1)

    def shutdown(self):
        rospy.signal_shutdown("SERVER DOWN")


if __name__ == '__main__':
    rospy.init_node('victimValidation')
    ValidateVictimActionServer('victimValidation')
    rospy.spin()
