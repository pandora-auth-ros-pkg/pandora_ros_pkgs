#!usr/bin/env python

import rospy
from pandora_testing_tools.testing_interface import alert_delivery
from pandora_testing_tools.mocks import mock_gui
from pandora_testing_tools.mocks import mock_navigation
from pandora_testing_tools.mocks import mock_explorer
from pandora_testing_tools.mocks import mock_end_effector_planner

class IntegrationTester():

    def __init__(self, mocking_navigation=True, mocking_exploration=True,
        mocking_end_effector_planner=True, mocking_gui=True):

        self.mocking_navigation = mocking_navigation
        self.mocking_exploration = mocking_exploration
        self.mocking_gui = mocking_gui
        self.mocking_end_effector_planner = mocking_end_effector_planner
        if self.mocking_navigation:
            self.navigation = mock_navigation.MockNavigation(
                move_base_topic = '/move_base')
            rospy.loginfo("[IntegrationTester] Movk navigation ok")
        if self.mocking_exploration:
            self.explorer = mock_explorer.MockExplorer(
                exploration_topic = '/do_exploration')
            rospy.loginfo("[IntegrationTester] Movk explorer ok")
        if self.mocking_gui:
            self.gui = mock_gui.MockGui(
                gui_validation_topic = '/gui/validate_victim')
            rospy.loginfo("[IntegrationTester] Movk gui ok")
        if self.mocking_end_effector_planner:
            self.end_effector_planner = mock_end_effector_planner.MockEndEffectorPlanner(
                end_effector_planner_topic = '/control/move_end_effector_planner_action')
            rospy.loginfo("[IntegrationTester] Movk end effector planner ok")
        self.delivery_boy = alert_delivery.AlertDeliveryBoy('headCamera')
        rospy.loginfo("[IntegrationTester] Alert deliverery boy ok")

    def __del__(self):

        if self.mocking_navigation:
            self.navigation.__del__()
        if self.mocking_exploration:
            self.explorer.__del__()
        if self.mocking_gui:
            self.gui.__del__()
        if self.mocking_end_effector_planner:
            self.end_effector_planner.__del__()

