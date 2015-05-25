#!/usr/bin/env python
import sys

import unittest

PKG = "pandora_stabilizer"
NAME = "PandoraStabilizerNodeTester"

import roslib
roslib.load_manifest(PKG)
import rostest
import rospy
import tf
from pandora_testing_tools.testing_interface import test_base
from pandora_sensor_msgs.msg import ImuRPY
from std_msgs.msg import Float64


class PandoraStabilizerNodeTester(test_base.TestBase):

    def setUp(self):
        super(PandoraStabilizerNodeTester, self).setUp()
        self.min_roll = rospy.get_param("/control/min_roll")
        self.max_roll = rospy.get_param("/control/max_roll")
        self.min_pitch = rospy.get_param("/control/min_pitch")
        self.max_pitch = rospy.get_param("/control/max_pitch")
        self.imu_topic = rospy.get_param("/control/imu_topic")
        self.roll_cmd_topic = rospy.get_param("/control/roll_command_topic")
        self.pitch_cmd_topic = rospy.get_param("/control/pitch_command_topic")

    def test_max_pitch(self):
        imu_msg = ImuRPY()
        imu_msg.roll = 0.0
        imu_msg.pitch = -1.0
        imu_msg.yaw = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic,
                                           imu_msg)
        self.assertLessEqual(output_data, self.max_pitch)

    def test_max_roll(self):
        imu_msg = ImuRPY()
        imu_msg.roll = -2.0
        imu_msg.pitch = 0.0
        imu_msg.yaw = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic,
                                           imu_msg)
        self.assertLessEqual(output_data, self.max_roll)

    def test_min_pitch(self):
        imu_msg = ImuRPY()
        imu_msg.roll = 0.0
        imu_msg.pitch = 2.0
        imu_msg.yaw = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_pitch)

    def test_min_roll(self):
        imu_msg = ImuRPY()
        imu_msg.roll = 2.0
        imu_msg.pitch = 0.0
        imu_msg.yaw = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_roll)

    def test_successive_roll_posts(self):
        imu_msg = ImuRPY()
        imu_msg.roll = -2.0
        imu_msg.pitch = 0.0
        imu_msg.yaw = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_roll)
        self.assertLessEqual(output_data, self.max_roll)

        imu_msg.roll = -1.0
        imu_msg.pitch = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_roll)
        self.assertLessEqual(output_data, self.max_roll)

        imu_msg.roll = -0.5
        imu_msg.pitch = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_roll)
        self.assertLessEqual(output_data, self.max_roll)

        imu_msg.roll = 0.0
        imu_msg.pitch = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_roll)
        self.assertLessEqual(output_data, self.max_roll)

        imu_msg.roll = 0.5
        imu_msg.pitch = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_roll)
        self.assertLessEqual(output_data, self.max_roll)

        imu_msg.roll = 1.0
        imu_msg.pitch = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_roll)
        self.assertLessEqual(output_data, self.max_roll)

        imu_msg.roll = 2.0
        imu_msg.pitch = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_roll)
        self.assertLessEqual(output_data, self.max_roll)

    def test_successive_pitch_posts(self):
        imu_msg = ImuRPY()
        imu_msg.roll = 0.0
        imu_msg.pitch = -2.0
        output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_pitch)
        self.assertLessEqual(output_data, self.max_pitch)

        imu_msg.roll = 0.0
        imu_msg.pitch = -1.0
        output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_pitch)
        self.assertLessEqual(output_data, self.max_pitch)

        imu_msg.roll = 0.0
        imu_msg.pitch = -0.5
        output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_pitch)
        self.assertLessEqual(output_data, self.max_pitch)

        imu_msg.roll = 0.0
        imu_msg.pitch = 0.0
        output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_pitch)
        self.assertLessEqual(output_data, self.max_pitch)

        imu_msg.roll = 0.0
        imu_msg.pitch = 0.5
        output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_pitch)
        self.assertLessEqual(output_data, self.max_pitch)

        imu_msg.roll = 0.0
        imu_msg.pitch = 1.0
        output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_pitch)
        self.assertLessEqual(output_data, self.max_pitch)

        imu_msg.roll = 0.0
        imu_msg.pitch = 2.0
        output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
        self.assertGreaterEqual(output_data, self.min_pitch)
        self.assertLessEqual(output_data, self.max_pitch)

    def mockTransaction(self, input_topic, output_topic, input_data):
        self.mockPublish(input_topic, output_topic, input_data)
        self.assertTrue(self.repliedList[output_topic])
        self.assertEqual(len(self.messageList[output_topic]), 1)
        output_data = self.messageList[output_topic].pop()
        return output_data.data

if __name__ == "__main__":
    subscriber_topics = [
        ("/laser_roll_controller/command", "std_msgs", "Float64"),
        ("/laser_pitch_controller/command", "std_msgs", "Float64")]
    publisher_topics = [("/sensors/imu_rpy", "pandora_sensor_msgs", "ImuRPY")]
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    PandoraStabilizerNodeTester.connect(subscriber_topics, publisher_topics, 2, False)
    rostest.rosrun(PKG, NAME, PandoraStabilizerNodeTester, sys.argv)
    PandoraStabilizerNodeTester.disconnect()
