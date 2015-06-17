#!/usr/bin/env python

""" Tests for the move end effector server functionality """

import unittest

import roslib
roslib.load_manifest('pandora_end_effector_controller')
import rospy

from rospy import Publisher
from std_msgs.msg import String
from pandora_end_effector_controller.effector_clients import SensorClient, LinearClient, HeadClient
from pandora_end_effector_controller.client_factory import ClientFactory
from pandora_end_effector_controller.move_end_effector_server import MoveEndEffectorServer
from pandora_end_effector_controller.mocks.goal_maker import EffectorGoalMaker
from actionlib import SimpleActionClient as Client
from actionlib import GoalStatus
from pandora_end_effector_controller.topics import move_end_effector_controller_topic, move_kinect_topic, \
    move_head_topic, move_linear_topic
from pandora_sensor_orientation_controller.msg import MoveSensorAction, MoveSensorGoal
from pandora_linear_movement_controller.msg import MoveLinearAction, MoveLinearGoal
from pandora_end_effector_controller.msg import MoveEndEffectorAction, MoveEndEffectorGoal


class TestInitialFunctions(unittest.TestCase):

  def setUp(self):
    rospy.init_node('test_move_end_effector_server_node')
    self.server = MoveEndEffectorServer()
    self.goal = MoveEndEffectorGoal()
    rospy.sleep(1)

  def test_init(self):
    ''' Makes sure the initialization went well '''
    self.assertEqual(type(self.server.factory), type(ClientFactory()))

    # The following exact values depend on the factory settings!
    self.assertEqual(len(self.server.current_clients), 3)
    self.assertEqual(type(self.server.current_clients[0]), type(SensorClient()))
    self.assertEqual(type(self.server.current_clients[1]), type(LinearClient()))
    self.assertEqual(type(self.server.current_clients[2]), type(HeadClient()))

  def test_fill_goals(self):
    ''' Tests if all clients' goals are filled successfully '''

    self.goal.command = MoveEndEffectorGoal.SCAN
    self.goal.point_of_interest = '0,0,0'
    self.goal.center_point = '0,0,1'
    self.server.current_goal = self.goal
    self.server.fill_goals()
    self.assertEqual(self.server.current_clients[0].goal.command , 4)
    self.assertEqual(self.server.current_clients[0].goal.point_of_interest , '0,0,0')
    self.assertEqual(self.server.current_clients[1].goal.command , 1)
    self.assertEqual(self.server.current_clients[1].goal.point_of_interest , '0,0,0')
    self.assertEqual(self.server.current_clients[1].goal.center_point ,'0,0,1')
    self.assertEqual(self.server.current_clients[2].goal.command , 4)
    self.assertEqual(self.server.current_clients[2].goal.point_of_interest , '0,0,0')

class TestFunctionsInvolvingClients(unittest.TestCase):

  def setUp(self):
    rospy.init_node('test_move_end_effector_server_node')
    self.effector_client = Client(move_end_effector_controller_topic, MoveEndEffectorAction)
    self.server = MoveEndEffectorServer()
    self.goal = MoveEndEffectorGoal()
    self.goal_maker = EffectorGoalMaker()
    self.sensor_pub = Publisher('mock/sensor', String)
    self.linear_pub = Publisher('mock/linear', String)
    self.head_pub = Publisher('mock/head', String)
    rospy.sleep(1)

  def test_send_goals(self):
    ''' Tests if all goals are sent to clients successfully '''

    self.server.current_goal = self.goal_maker.create_goal()
    self.server.fill_goals()
    self.server.send_goals()
    rospy.sleep(1)
    for client in self.server.current_clients:
        self.assertEqual(client.client.get_state(), GoalStatus.ACTIVE)

  def test_wait_for_result(self):
    ''' Tests if all clients wait for result '''

    self.server.current_goal = self.goal_maker.create_goal()
    self.server.fill_goals()
    self.server.send_goals()
    self.server.wait_for_result()
    for client in self.server.current_clients:
      self.assertNotEqual(client.client.get_state(), GoalStatus.ACTIVE)
      self.assertNotEqual(client.client.get_state(), GoalStatus.PENDING)

  def test_result_success(self):
    ''' Tests if the server is successfully set to succeeded  '''


    self.sensor_pub.publish('success:1')
    self.linear_pub.publish('success:1')
    self.head_pub.publish('success:1')
    self.server.current_goal = self.goal_maker.create_goal()
    self.server.fill_goals()
    self.server.send_goals()
    self.server.wait_for_result()
    self.assertTrue(self.server.check_succeeded())

  def test_result_aborted(self):
    ''' Tests if the server is successfully set to aborted  '''

    self.sensor_pub.publish('abort:1')
    self.linear_pub.publish('success:1')
    self.head_pub.publish('success:1')
    self.server.current_goal = self.goal_maker.create_goal()
    self.server.fill_goals()
    self.server.send_goals()
    self.server.wait_for_result()
    self.assertTrue(self.server.check_aborted())

  def test_result_preempted(self):
    ''' Tests if the server is successfully set to preempted  '''

    self.sensor_pub.publish('preempt:1')
    self.linear_pub.publish('success:1')
    self.head_pub.publish('success:1')
    self.server.current_goal = self.goal_maker.create_goal()
    self.server.fill_goals()
    self.server.send_goals()
    self.server.wait_for_result()
    self.assertTrue(self.server.check_preempted())    

class TestClientsAndServer(unittest.TestCase):

  def setUp(self):
    rospy.init_node('test_move_end_effector_server_node')
    self.effector_client = Client(move_end_effector_controller_topic, MoveEndEffectorAction)
    self.server = MoveEndEffectorServer()
    self.goal_maker = EffectorGoalMaker()
    self.sensor_pub = Publisher('mock/sensor', String, queue_size=1)
    self.linear_pub = Publisher('mock/linear', String, queue_size=1)
    self.head_pub = Publisher('mock/head', String, queue_size=1)
    rospy.sleep(1)

  def test_execute_cb(self):
    ''' Tests if server is set correctly (triggered by callback!) '''

    self.sensor_pub.publish('success:1')
    self.linear_pub.publish('success:1')
    self.head_pub.publish('success:1')
    self.goal = MoveEndEffectorGoal()
    self.goal = self.goal_maker.create_goal()
    self.server.wait_for_servers()
    self.effector_client.send_goal(self.goal)
    self.effector_client.wait_for_result()
    self.assertEqual(self.effector_client.get_state(), GoalStatus.SUCCEEDED)
    self.sensor_pub.publish('abort:1')
    self.linear_pub.publish('success:1')
    self.head_pub.publish('success:1')
    self.goal = MoveEndEffectorGoal()
    self.goal = self.goal_maker.create_goal()
    self.effector_client.send_goal(self.goal)
    self.effector_client.wait_for_result()
    self.assertEqual(self.effector_client.get_state(), GoalStatus.ABORTED)
    self.sensor_pub.publish('preempt:1')
    self.linear_pub.publish('success:1')
    self.head_pub.publish('preempt:1')
    self.goal = MoveEndEffectorGoal()
    self.goal = self.goal_maker.create_goal()
    self.effector_client.send_goal(self.goal)
    self.effector_client.wait_for_result()
    self.assertEqual(self.effector_client.get_state(), GoalStatus.PREEMPTED)
    self.sensor_pub.publish('abort:1')
    self.linear_pub.publish('success:1')
    self.head_pub.publish('preempt:1')
    self.goal = MoveEndEffectorGoal()
    self.goal = self.goal_maker.create_goal()
    self.effector_client.send_goal(self.goal)
    self.effector_client.wait_for_result()
    self.assertEqual(self.effector_client.get_state(), GoalStatus.ABORTED)
