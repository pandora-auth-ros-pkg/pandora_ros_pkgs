from cost_functions.goal_cost import GoalCost
from cost_functions.trajectory_cost import TrajectoryCost
from cost_functions.cost_graph.fuse_cost_node import FuseCostNode
from cost_functions.linear_fusion import LinearFusion

from pandora_kinodynamic_control import utils
from pandora_kinodynamic_control.params import *

class MotionReward(object):

    """ @brief: It is responsible for using the environment to calculate
        the reward for execution of a task of an agent

        It uses cost functions which are combined in a directed acyclic graph
        in order to produce a unified cost that an action caused.
        It is also delegated with handling information from the environment.

    """

    def __init__(self):
        """ @brief: Initialization of a MotionReward object"""
        self.goal_cost_node = GoalCost()
        self.trajectory_cost_node = TrajectoryCost()
        self.cost_nodes = [self.goal_cost_node, self.trajectory_cost_node]
        self.strategy = LinearFusion()
        self.fuse_cost_node = FuseCostNode(self.cost_nodes, self.strategy)
        self._time_granularity = None

        # Cost_to_Reward function Parameters
        self._max_reward = None
        # If total_cost > cost_threshold , then reward shall be negatives
        self._cost_threshold = None

    def get_reward_from_pose(self, actual_pose):
        """ @brief: Returns and calculates the reward associated with a certain
            action taken by the agent who controls the kinodynamic controller

        @param actual_pose: new information from the environment
        considering the latest action taken
        @type actual_pose: Pose
        @return: double, the reward of the action

        """
        self.update_actual_info(actual_pose)
        self.goal_cost_node.set_actual_pose(actual_pose)

        return self.__get_reward()

    def get_reward_from_trajectory(self, actual_trajectory):
        """ @brief: Returns and calculates the reward associated with a certain
            action taken by the agent who controls the kinodynamic controller

            Need vehicle's actual trajectory as given from the corresponding topic.

        @param actual_trajectory: actual trajectory of vehicle
        between two time moments of consecutive states
        @type actual_trajectory: list of Pose
        @return: double, the reward of an action

        """
        self.trajectory_cost_node.set_actual_trajectory(actual_trajectory)
        if len(actual_trajectory) <= 1:
            print "[MotionReward] ERROR: actual_trajectory has length <= 1"
        self.goal_cost_node.set_actual_pose(actual_trajectory[-1])

        return self.__get_reward()

    def __get_reward(self):
        """ @brief: Returns reward according to the unified cost return by the
            cost functions

        @return: double, the reward of an action

        """
        self.goal_cost_node.update_cost()
        self.trajectory_cost_node.update_cost()
        self.fuse_cost_node.update_cost()

        cost = self.fuse_cost_node.get_cost()
        reward = self.cost_to_reward(cost)
        # Clear Trajectories from trajectory node.
        self.trajectory_cost_node.clear_trajectories()
        return reward

    def init_info_from_expected(self, expected_trajectory, clear=False):
        """ @brief: Initialize cost functions with expected trajectory given from
            local planner

        @param expected_trajectory: the expected trajectory
        @type expected_trajectory: list of Pose
        @return: nothing

        """
        # # Reset Case:
        # if clear:
        #     self.trajectory_cost_node.clear_trajectories()
        self.trajectory_cost_node.extend_expected_trajectory(expected_trajectory)

        # Checks trajectory
        if len(expected_trajectory) <= 1:
            print "[MotionReward] ERROR: expected_trajectory has length <= 1"

        # Goal Related
        goal_pose = expected_trajectory[-1]
        self.goal_cost_node.set_goal_pose(goal_pose)

    def set_params(self, cost_weights, time_granularity,max_reward,cost_threshold):
        """ @brief: Configure parameters of cost nodes

        @param cost_weights: contains weights for the linear fusion of the costs
        @type cost_weights: list of double
        @param time_granularity: resolution of a discrete curve in respect
        to the time
        @type time_granularity: double
        @return: nothing

        """
        self.strategy.set_weights(cost_weights)
        self._time_granularity = time_granularity
        self._max_reward = max_reward
        self._cost_threshold = cost_threshold

    def cost_to_reward(self,cost):
        """ @brief: Function to transform cost to reward.

        @param cost: The result of linear fusion of cost functions of each module.
        @type cost: double
        @return : The total reward
        @type : double
        @note: Obvisouly this function must be a strictly decreasing function.
        In this implementation we use a linear function with negative slope
        """
        return -(self._max_reward/self._cost_threshold)*cost + self._max_reward
