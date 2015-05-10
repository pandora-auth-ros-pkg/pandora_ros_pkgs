#!/usr/bin/env python
import sys

import unittest

PKG = "pandora_stabilizer"
NAME = "PandoraStabilizerNodeTester"

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import tf
from pandora_testing_tools.testing_interface import test_base
from sensor_msgs.msg import Imu
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
    imu_msg = self.rpy_to_imu_msg(0.0, -1.0, 0.0)
    output_data = self.mockTransaction(
      self.imu_topic, self.pitch_cmd_topic, imu_msg)
    self.assertLessEqual(output_data, self.max_pitch)

  def test_max_roll(self):
    imu_msg = self.rpy_to_imu_msg(-2.0, 0.0, 0.0)
    output_data = self.mockTransaction(
      self.imu_topic, self.roll_cmd_topic, imu_msg)
    self.assertLessEqual(output_data, self.max_roll)

  def test_min_pitch(self):
    imu_msg = self.rpy_to_imu_msg(0.0, 2.0, 0.0)
    output_data = self.mockTransaction(
      self.imu_topic, self.pitch_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_pitch)

  def test_min_roll(self):
    imu_msg = self.rpy_to_imu_msg(2.0, 0.0, 0.0)
    output_data = self.mockTransaction(
      self.imu_topic, self.roll_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_roll)

  def test_successive_roll_posts(self):
    imu_msg = self.rpy_to_imu_msg(-2.0, 0.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_roll)
    self.assertLessEqual(output_data, self.max_roll)


    imu_msg = self.rpy_to_imu_msg(-1.0, 0.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_roll)
    self.assertLessEqual(output_data, self.max_roll)

    imu_msg = self.rpy_to_imu_msg(-0.5, 0.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_roll)
    self.assertLessEqual(output_data, self.max_roll)

    imu_msg = self.rpy_to_imu_msg(0.0, 0.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_roll)
    self.assertLessEqual(output_data, self.max_roll)

    imu_msg = self.rpy_to_imu_msg(0.5, 0.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_roll)
    self.assertLessEqual(output_data, self.max_roll)

    imu_msg = self.rpy_to_imu_msg(1.0, 0.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_roll)
    self.assertLessEqual(output_data, self.max_roll)

    imu_msg = self.rpy_to_imu_msg(2.0, 0.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.roll_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_roll)
    self.assertLessEqual(output_data, self.max_roll)

  def test_successive_pitch_posts(self):
    imu_msg = self.rpy_to_imu_msg(0.0, -2.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_pitch)
    self.assertLessEqual(output_data, self.max_pitch)

    imu_msg = self.rpy_to_imu_msg( 0.0, -1.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_pitch)
    self.assertLessEqual(output_data, self.max_pitch)

    imu_msg = self.rpy_to_imu_msg( 0.0, -0.5, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_pitch)
    self.assertLessEqual(output_data, self.max_pitch)

    imu_msg = self.rpy_to_imu_msg(0.0, 0.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_pitch)
    self.assertLessEqual(output_data, self.max_pitch)

    imu_msg = self.rpy_to_imu_msg(0.0, 0.5, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_pitch)
    self.assertLessEqual(output_data, self.max_pitch)

    imu_msg = self.rpy_to_imu_msg(0.0, 1.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_pitch)
    self.assertLessEqual(output_data, self.max_pitch)

    imu_msg = self.rpy_to_imu_msg(0.0, 2.0, 0.0)
    output_data = self.mockTransaction(self.imu_topic, self.pitch_cmd_topic, imu_msg)
    self.assertGreaterEqual(output_data, self.min_pitch)
    self.assertLessEqual(output_data, self.max_pitch)


  def rpy_to_imu_msg(self, roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(
      roll, pitch, yaw)
    imu_msg = Imu()
    imu_msg.orientation.x = quaternion[0]
    imu_msg.orientation.y = quaternion[1]
    imu_msg.orientation.z = quaternion[2]
    imu_msg.orientation.w = quaternion[3]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return imu_msg

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
  publisher_topics = [("/sensors/imu", "sensor_msgs", "Imu")]
  rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
  PandoraStabilizerNodeTester.connect(
    subscriber_topics, publisher_topics, 2, False)
  rostest.rosrun(PKG, NAME, PandoraStabilizerNodeTester, sys.argv)
  PandoraStabilizerNodeTester.disconnect()
