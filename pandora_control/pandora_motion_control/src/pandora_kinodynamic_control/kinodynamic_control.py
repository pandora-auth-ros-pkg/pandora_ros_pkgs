#!/usr/bin/env python
import os
import pickle
import rospy
import scipy
import matplotlib.pyplot
from threading import Thread

from pandora_kinodynamic_control.dataset_utils import *
from pandora_kinodynamic_control.experiment import Experiment
from pandora_kinodynamic_control.navigation_task import NavigationTask
from pandora_kinodynamic_control.navigation_environment import NavigationEnvironment
from pybrain.rl.learners.valuebased import ActionValueTable
from pybrain.rl.agents import LearningAgent
from pybrain.rl.learners import SARSA
from pybrain.rl.explorers import EpsilonGreedyExplorer
from pandora_kinodynamic_control.params import *

from pandora_motion_control.srv import StoreAVTable
from pandora_motor_hardware_interface.msg import KinematicParameters

class KinodynamicController(object):
    """ @brief: A class to handle interactions between various components of the RL module"""

    def __init__(self):
        """ @brief: Setting up internal parameters for the RL module"""

        # Navigation Task
        self._environment = NavigationEnvironment()
        self._task = NavigationTask(self._environment)

        # Number of States : (read from params.py)
        self._states = STATES
        self._state_limits = LIMITS

        # Total number of states:
        self._number_of_states = 1
        for i in self._states:
            self._number_of_states *= i

        # Number of actions
        self._actions = ACTION_STATES
        self._action_limits = ACTION_RANGE

        # Action Value Table directory
        self.tables_directory = os.path.dirname(__file__) + "/tables/"
        self.table_code = "S"+str(self._number_of_states)+"_"+"A"+str(self._actions)
        self._filename = FILENAME + self.table_code

        # Action Value Table setup
        self.load_AV_Table()

        # Declare ROS Service to store Action Value Table
        store_service = rospy.Service('store_table', StoreAVTable, self.store_cb)

        # Set up task parameters:
        self._task.set_params(COMMAND_DURATION,
                              FUSION_WEIGHTS,
                              TIME_GRANULARITY,
                              self._state_limits,
                              MAX_REWARD,
                              COST_THRESHOLD)

        # Agent set up
        self._learner = SARSA(alpha,gamma)
        self._learner._setExplorer(EpsilonGreedyExplorer(epsilon))
        self._agent = LearningAgent(self._av_table, self._learner)

        # Experiment set up
        self._experiment = Experiment(self._task,self._agent)
        self._experiment.set_params(STEP_SIZE)

        # Start print table thread
        if VISUALIZATION is True:
            try:
                #thread.start_new_thread(self.print_table,())
                self.visualization_thread = Thread(target = self.print_table, args = () )
                self.visualization_thread.start()
            except:
                print "Failed to start visualization thread!"

        print "Successfully Initialization of RL module! (kappa)"


    def store_cb(self,req):
        storeData(self._av_table,self._filename)
        print "Saved AV Table"
        return True

    def load_AV_Table(self):

        load_D = loadData(self._filename)
        if load_D[1] == True:
            self._av_table = load_D[0]
            print "Found Table!"

        else:
            self._av_table = ActionValueTable(self._number_of_states, self._actions)
            self._av_table.initialize(0.0)
            print "No training for this format. Creating new AV table"

    def print_table(self):
        """ @brief: Visual Representation of Action Value Table

        @return: nothing

        """
        matplotlib.pyplot.ion()
        while True:
            data = self._av_table.params.reshape(self._number_of_states,self._actions)
            matplotlib.pyplot.pcolor(data,
                                     cmap = matplotlib.pyplot.cm.RdYlGn,
                                     vmin = -MAX_REWARD,
                                     vmax = MAX_REWARD)
            matplotlib.pyplot.draw()
            rospy.sleep(2)

    def __del__(self):
        # Terminate visualization thread
        if VISUALIZATION is True:
            self.visualization_thread.join()

        # Copy learned data to repo
        if add_to_repo():
            print "Copied data to pandora_motion_control"
        else:
            pass

def main():
    # Spawn ROS node , create controller and spin!
    rospy.init_node('kinodynamic_controller')
    controller = KinodynamicController()
    rospy.spin()
    # try:
    #     # Spawn ROS node , create controller and spin!
    #     rospy.init_node('kinodynamic_controller')
    #     controller = KinodynamicController()
    #     rospy.spin()
    # except:
    #     # Case when RL modules fails for some reason , reset velocity controller
    #     # to nomral control mode (all parameters set to 1)
    #     pub = rospy.Publisher(COMMAND_TOPIC, KinematicParameters, queue_size=1)
    #
    #     # Fill reset msg
    #     params = KinematicParameters()
    #     params.scale_left = 1
    #     params.scale_right = 1
    #     params.terrain_param = 1
    #
    #     pub.publish(params)
    #     print "RL module failed.Switching to controller-only mode"

if __name__ == '__main__':
    main()
    # try:
    #     # Spawn ROS node , create controller and spin!
    #     rospy.init_node('kinodynamic_controller')
    #     controller = KinodynamicController()
    #     rospy.spin()
    # except:
    #     # Case when RL modules fails for some reason , reset velocity controller
    #     # to nomral control mode (all parameters set to 1)
    #     pub = rospy.Publisher(COMMAND_TOPIC, KinematicParameters, queue_size=1)
    #
    #     # Fill reset msg
    #     params = KinematicParameters()
    #     params.scale_left = 1
    #     params.scale_right = 1
    #     params.terrain_param = 1
    #
    #     pub.publish(params)
    #     print "RL module failed.Switching to controller-only mode"
