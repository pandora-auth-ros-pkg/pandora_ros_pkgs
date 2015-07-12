from cost_graph.cost_node import CostNode
from pandora_kinodynamic_control.utils import hausdorff_distance

class TrajectoryCost(CostNode):

    """ @brief: Implementation of CostNode for calculate a cost from trajectory
        dissimilarity errors"""

    def __init__(self):
        """ @brief: Initiates a TrajectoryCost object"""
        super(TrajectoryCost, self).__init__()
        self.expected_trajectory = list()
        self.actual_trajectory = list()

    def update_cost(self):
        """ @brief: Uses expected and actual trajectory (list of poses) in order
            to calculate a cost based on their dissimilarity

            Uses a specific (which?) curve similarity algorithm.
            Updates self._cost.

        @return: nothing

        """
        hausdorff_A = hausdorff_distance(self.expected_trajectory,self.actual_trajectory )
        hausdorff_B = hausdorff_distance(self.actual_trajectory,self.expected_trajectory)
        self._cost = max(hausdorff_A,hausdorff_B)

    def append_actual_pose(self, pose):
        """ @brief: Appends an actual pose to the self.actual_trajectory
            discrete curve

            Should be called in respect to the time_granularity set in the
            TrajectoryCost object.

        @param pose: vehicle's actual pose the moment that granularity
        period has ended
        @type pose: tuple of doubles (x, y, yaw)
        @return: nothing

        """
        self.actual_trajectory.append(pose)

    def set_actual_trajectory(self, trajectory):
        """ @brief: Setter for the actual trajectory of the robot
            among the time instance that the movement command was issued
            and the time instance the movement command has finished
            (after a given duration of time)

            To be called in case there is a mechanism that tracks the vehicle's
            trajectory.

        @param trajectory: vehicle's actual trajectory between 2 moments of time
        @type trajectory: list of tuples (x, y, yaw)
        @return: nothing

        """
        self.actual_trajectory = trajectory

    def get_actual_trajectory(self):
        """ @brief: Getter for the actual trajectory done

        @return: list of tuples (x, y, yaw), the actual trajectory

        """
        return self.actual_trajectory

    def extend_expected_trajectory(self, trajectory):
        """ @brief: Extends self.expected_trajectory with  the expected
            trajectory of the robot as calculated by the local planner

        @param trajectory: the expected trajectory
        @type trajectory: list of tuples (x, y, yaw)
        @return: nothing

        """
        self.expected_trajectory.extend(trajectory)

    def get_expected_trajectory(self):
        """ @brief: Getter for the expected trajectory calculated

        @return: list of tuples (x, y, yaw), the expected trajectory

        """
        return self.expected_trajectory

    def clear_trajectories(self):
        """ @brief: Reset actual and expected trajectories to empty lists

        @return: nothing

        """
        self.actual_trajectory = list()
        self.expected_trajectory = list()
