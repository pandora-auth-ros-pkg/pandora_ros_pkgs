#!/usr/bin/env python
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
    rospy.init_node('victimValidation')
    ValidateVictimActionServer('victimValidation')
    rospy.spin()
