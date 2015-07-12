from cost_graph.cost_node import CostNode
from geometry_msgs.msg import Pose

from pandora_kinodynamic_control.utils import find_distance
from pandora_kinodynamic_control.params import *

class GoalCost(CostNode):

    """ @brief: Implementation of CostNode for calculating a cost from goal
        based errors"""

    def __init__(self):
        """ @brief: Initiates a GoalCost object"""
        super(GoalCost, self).__init__()
        self.expected_pose = Pose()
        self.actual_pose = Pose()

    def update_cost(self):
        """ @brief: Uses expected final pose (goal) and actual final pose in
            order to calculate a cost. Updates self._cost

        @return: nothing

        """
        # Relative Reward mode:
        point_a = (self.expected_pose[0],self.expected_pose[1])
        point_b = (self.actual_pose[0],self.actual_pose[1])

        diff_distance = find_distance(point_a,point_b)
        diff_angle = abs(self.expected_pose[2]-self.actual_pose[2])

        max_distance = COMMAND_DURATION * LINEAR_LIMITS[1]
        max_angle    = COMMAND_DURATION * ANGULAR_LIMITS[1]

        self._cost = diff_distance/max_distance + diff_angle/max_angle

        # Absolute Rewards:   (BEWARE : requires retuning of cost_to_reward function)
        #self._cost = find_distance(self.expected_pose, self.actual_pose)

    def set_goal_pose(self, pose):
        """ @brief: Sets goal pose, expected pose of vehicle when movement
            command will have finished

        @param pose: expected final vehicle's pose (we care about x, y and yaw)
        @type pose: Pose
        @return: nothing

        """
        self.expected_pose = pose

    def set_actual_pose(self, pose):
        """ @brief: Sets final pose, actual pose of vehicle when movement
            command has finished

        @param pose: actual final vehicle's pose (we care about x, y and yaw)
        @type pose: Pose
        @return: nothing

        """
        self.actual_pose = pose
