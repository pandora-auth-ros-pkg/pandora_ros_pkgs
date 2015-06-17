#!/usr/bin/env python

""" Helping mock to create MoveEndEffectorGoals """

from pandora_end_effector_controller.msg import MoveEndEffectorGoal
import random

class EffectorGoalMaker(object):

  def __init__(self):
    self.goal = MoveEndEffectorGoal()
    self.commands = [MoveEndEffectorGoal.TEST, MoveEndEffectorGoal.PARK, 
                     MoveEndEffectorGoal.TRACK, MoveEndEffectorGoal.LAX_TRACK,
                     MoveEndEffectorGoal.SCAN]

  def create_goal(self, command=None,
                  point_of_interest=None,
                  center_point=None):

    self.goal.command = command if command else random.choice(self.commands)
    self.goal.point_of_interest = point_of_interest if point_of_interest else '0,0,1'
    self.goal.center_point = center_point if center_point else '0,1,0'
    return self.goal
    