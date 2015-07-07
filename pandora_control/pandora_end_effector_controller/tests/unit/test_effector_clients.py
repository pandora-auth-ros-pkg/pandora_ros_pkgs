#!/usr/bin/env python

""" Tests for the client factory functionality """

import unittest

import roslib
roslib.load_manifest('pandora_end_effector_controller')
import rospy

from rospy import Publisher
from std_msgs.msg import String
from pandora_end_effector_controller.effector_clients import SensorClient, LinearActuatorClient, HeadClient
from pandora_end_effector_controller.client_factory import ClientFactory
from actionlib import SimpleActionClient as Client
from actionlib import GoalStatus
from pandora_end_effector_controller.topics import move_end_effector_controller_topic, move_kinect_topic, \
    move_head_topic, move_linear_actuator_topic
from pandora_sensor_orientation_controller.msg import MoveSensorAction, MoveSensorGoal
from pandora_linear_actuator_controller.msg import MoveLinearActuatorAction, MoveLinearActuatorGoal
from pandora_end_effector_controller.msg import MoveEndEffectorAction, MoveEndEffectorGoal

class TestSensorClient(unittest.TestCase):

  def setUp(self):
    rospy.init_node('effector_clients_test')
    self.sensor_pub = Publisher('/mock/sensor', String)
    self.client_instance = SensorClient()
    self.effector_goal = MoveEndEffectorGoal()
    self.effector_goal.command = 'TEST'
    self.effector_goal.point_of_interest = '0,0,1'
    self.effector_goal.center_point = '0,1,0'
    rospy.sleep(1)

  def test_init(self):
    self.assertEqual(type(self.client_instance.client), type(Client(move_kinect_topic, MoveSensorAction)))
    # Following three must be in accordance to client_dict
    self.assertEqual(type(self.client_instance.goal), type(MoveSensorGoal()))
    self.assertEqual(self.client_instance.topic, move_kinect_topic)
    self.assertEqual(type(self.client_instance.action), type(MoveSensorAction))

  def test_fill_goal(self):
    self.client_instance.fill_goal(self.effector_goal)
    self.assertEqual(self.client_instance.goal.command, 0)
    self.assertEqual(self.client_instance.goal.point_of_interest, '0,0,1')

  def test_send_goal(self):
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    rospy.sleep(1)
    self.assertEqual(self.client_instance.client.get_state(), GoalStatus.ACTIVE)

  def test_has_succeeded(self):
    self.sensor_pub.publish(String('success:1'))
    rospy.sleep(1)
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    self.client_instance.wait_result()
    self.assertTrue(self.client_instance.has_succeeded())

  def test_has_aborted(self):
    self.sensor_pub.publish(String('abort:1'))
    rospy.sleep(1)
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    self.client_instance.wait_result()
    self.assertTrue(self.client_instance.has_aborted())

  def test_has_preempted(self):
    self.sensor_pub.publish(String('preempt:1'))
    rospy.sleep(1)
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    self.client_instance.wait_result()
    self.assertTrue(self.client_instance.has_preempted())

class TestLinearActuatorClient(unittest.TestCase):

  def setUp(self):
    rospy.init_node('effector_clients_test')
    self.linear_actuator_pub = Publisher('/mock/linear_actuator', String)
    self.client_instance = LinearActuatorClient()
    self.effector_goal = MoveEndEffectorGoal()
    self.effector_goal.command = 'TEST'
    self.effector_goal.point_of_interest = '0,0,1'
    self.effector_goal.center_point = '0,1,0'
    self.effector_goal.center_point = '0,1,0'
    rospy.sleep(1)

  def test_init(self):
    self.assertEqual(type(self.client_instance.client), type(Client(move_linear_actuator_topic, MoveLinearActuatorAction)))
    # Following three must be in accordance to client_dict
    self.assertEqual(type(self.client_instance.goal), type(MoveLinearActuatorGoal()))
    self.assertEqual(self.client_instance.topic, move_linear_actuator_topic)
    self.assertEqual(type(self.client_instance.action), type(MoveLinearActuatorAction))

  def test_fill_goal(self):
    self.client_instance.fill_goal(self.effector_goal)
    self.assertEqual(self.client_instance.goal.command, 0)
    self.assertEqual(self.client_instance.goal.point_of_interest, '0,0,1')
    self.assertEqual(self.client_instance.goal.center_point, '0,1,0')

  def test_send_goal(self):
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    rospy.sleep(1)
    self.assertEqual(self.client_instance.client.get_state(), GoalStatus.ACTIVE)

  def test_has_succeeded(self):
    self.linear_actuator_pub.publish(String('success:1'))
    rospy.sleep(1)
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    self.client_instance.wait_result()
    self.assertTrue(self.client_instance.has_succeeded())

  def test_has_aborted(self):
    self.linear_actuator_pub.publish(String('abort:1'))
    rospy.sleep(1)
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    self.client_instance.wait_result()
    self.assertTrue(self.client_instance.has_aborted())

  def test_has_preempted(self):
    self.linear_actuator_pub.publish(String('preempt:1'))
    rospy.sleep(1)
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    self.client_instance.wait_result()
    self.assertTrue(self.client_instance.has_preempted())

class TestHeadClient(unittest.TestCase):

  def setUp(self):
    rospy.init_node('effector_clients_test')
    self.head_pub = Publisher('/mock/head', String)
    self.client_instance = HeadClient()
    self.effector_goal = MoveEndEffectorGoal()
    self.effector_goal.command = 'TEST'
    self.effector_goal.point_of_interest = '0,0,1'
    rospy.sleep(1)

  def test_init(self):
    self.assertEqual(type(self.client_instance.client), type(Client(move_head_topic, MoveSensorAction)))
    # Following three must be in accordance to client_dict
    self.assertEqual(type(self.client_instance.goal), type(MoveSensorGoal()))
    self.assertEqual(self.client_instance.topic, move_head_topic)
    self.assertEqual(type(self.client_instance.action), type(MoveSensorAction))

  def test_fill_goal(self):
    self.client_instance.fill_goal(self.effector_goal)
    self.assertEqual(self.client_instance.goal.command, 0)
    self.assertEqual(self.client_instance.goal.point_of_interest, '0,0,1')

  def test_send_goal(self):
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    rospy.sleep(1)
    self.assertEqual(self.client_instance.client.get_state(), GoalStatus.ACTIVE)

  def test_has_succeeded(self):
    self.head_pub.publish(String('success:1'))
    rospy.sleep(1)
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    self.client_instance.wait_result()
    self.assertTrue(self.client_instance.has_succeeded())

  def test_has_aborted(self):
    self.head_pub.publish(String('abort:1'))
    rospy.sleep(1)
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    self.client_instance.wait_result()
    self.assertTrue(self.client_instance.has_aborted())

  def test_has_preempted(self):
    self.head_pub.publish(String('preempt:1'))
    rospy.sleep(1)
    self.client_instance.wait_server()
    self.client_instance.fill_goal(self.effector_goal)
    self.client_instance.send_goal()
    self.client_instance.wait_result()
    self.assertTrue(self.client_instance.has_preempted())
