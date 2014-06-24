#!usr/bin/env python

import rospy
from pandora_testing_tools.testing_interface import alert_delivery
from pandora_testing_tools.mocks import mock_gui
from pandora_testing_tools.mocks import mock_navigation

#rospy.init_node("Mocking_mockers")
class IntegrationTester():

    def __init__(self):

        self.deliverer = alert_delivery.AlertDeliveryBoy()
        self.gui = mock_gui.MockGui("/gui/validate_victim")
        self.navi = mock_navigation.MockNavigation("/navigation/do_exploration", "/move_base")
