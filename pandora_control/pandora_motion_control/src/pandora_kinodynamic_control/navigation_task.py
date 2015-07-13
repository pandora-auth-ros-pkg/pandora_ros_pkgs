from pybrain.rl.environments.task import Task
from geometry_msgs.msg import Twist
from pandora_kinodynamic_control.motion_reward import MotionReward
from pandora_kinodynamic_control import utils
from params import *
from scipy import array

class NavigationTask(Task):

    """ @brief: Class that models the navigational task that we want a robotic
        vehicle to do

        Gets information from environment in order to determine vehicle's state.
        Formats actions to messages and send them to vehicle's kinodynamic model
        component.
        Gets the reward for agent's latest action.

    """

    def __init__(self, environment):
        """ @brief: Initialization for a NavigationTask object

        @param environment: An object that handles communication from and to
        this module's system environment
        @note : Although not regiestered as a class variable , NavigationTask holds
        a NavigationEnvironment object . The NavigationEnvironment is used for communication
        between the RL module and the real world.
        @type environment: NavigationEnvironment

        """

        super(NavigationTask, self).__init__(environment)

        self.motion_reward = MotionReward()

        # Navigation Command related
        self._cmd_vel = Twist()
        self._expected_trajectory = None
        self._actual_trajectory = None

        # Parameters
        self._trajectory_duration = None
        self._time_granularity = None

        # Discretization Parameters
        # (list containing number of states to produce in each element of sensros list)
        self._number_of_states = STATES

        # Flags
        self._updated_SLAM = False

        # State Processing:
        self._normalize_states = True
        self._discretize_states = True

    def set_velocity_command(self, cmd_vel):
        """ @brief: Setter for velocity command from navigation

            Calculating expected trajectory and thus getting action's reward
            depends from it. Should be called before self.getObservation

        @param cmd_vel: navigation's velocity command of center of mass
        @type cmd_vel: Twist
        @return: nothing

        """
        self._cmd_vel = cmd_vel

    def get_observation(self, final=False):
        """ @brief: Calculates environmental info and informs about vehicle's
            state in environment

            Delegates to NavigationEnvironment getSensors

        @override: from Task
        @return: vehicle's state, thus agent's state in MDP

        """
        # 1) Read Info from environment
        # Find current pose, return pitch, roll denormalized states
        [sensors ,curr_pose] = self.env.get_sensors()

        # 2) Trajectory Related:
        # Get actual trajectory as resulted from last command
        if final:
            update_trajectory = self.env.find_actual_trajectory()

            # Read Data:
            self._actual_trajectory = update_trajectory[0]
            self._updated_SLAM = update_trajectory[1]


        # Set latest action's expected trajectory in motion reward object
        if self._expected_trajectory is not None:
             self.motion_reward.init_info_from_expected(self._expected_trajectory)

        # Construct current expected trajectory from current pose, velocity
        # command and trajectory duration in time
        self._expected_trajectory = utils.calculate_expected_trajectory(curr_pose,self._cmd_vel,
                                                                        self._trajectory_duration, self._time_granularity)

        # 3)Retruns:
        if not final:
            return [list(),True]  # returns True so that callback can continue process

        # Make a state vector of (roll,pitch, linear, angular)
        sensors.append(self._cmd_vel.linear.x)
        sensors.append(self._cmd_vel.angular.z)

        sensors = utils.clamp(sensors,self.sensor_limits)
        if self._normalize_states:
            sensors = self.normalize(sensors)

            # Discretization can be applied only to normalized values
            if self._discretize_states:
                sensors = self.discretize(sensors)

        # Map sub-states to a total state:
        total_state = utils.state_mapper(tuple(sensors),self._number_of_states)
        return [array([total_state]),self._updated_SLAM]

    def get_reward(self):
        """ @brief: Calculates using MotionReward last action's reward
            and returns it

        @override: from Task
        @return: double, agent's latest action's reward

        """
        return self.motion_reward.get_reward_from_trajectory(self._actual_trajectory)

    def perform_action(self, action):
        """ @brief: Delegates to NavigationEnvironment to create messages and
            order a change in environment according to action argument

        @param action: agent's latest action
        @type action: integer
        @override: from Task
        @return: nothing
        @note : In current implementation agent's actions must be transformed in
        this function.

        """
        # Decode Agent's last action
        decoded_actions = utils.decode_actions(action,ACTIONS)

        # Transform every action
        transformed_actions = []
        for i in range(len(decoded_actions)):
            transformed_actions.append(utils.transform_action(decoded_actions[i],ACTIONS[i],ACTION_LIMITS[i]))

        # Delegate interaction with environment to NavigationEnvironment
        self.env.perform_action(transformed_actions)

    def discretize(self, sensors):
        """ @brief: Map continuous values to discrete values

        @param sensors: sensor values which define a state in continuous values
        @type sensors: list of doubles
        @return: list of doubles, sensor values but in a discrete set

        """
        discretized_sensors = []

        for i in range(len(sensors)):
            k = utils.discretize_value(sensors[i],self._number_of_states[i])
            discretized_sensors.append(k)

        return discretized_sensors

    def set_params(self, duration, cost_weights, time_granularity,
                  limits,max_reward,cost_threshold):
        """ @brief: Configure parameters for NavigationTask

        @param duration: duration between 2 supposed state changes, length of
        trajectories in time
        @type duration: double
        @param cost_weights: contains weights for the linear fusion of the costs
        @type cost_weights: list of double
        @param time_granularity: resolution of a discrete curve in respect
        to the time
        @type time_granularity: double
        @return: nothing

        """
        # Task Related
        self._trajectory_duration = duration
        self._time_granularity = time_granularity

        # Reward Related
        self.motion_reward.set_params(cost_weights,
                                      time_granularity,
                                      max_reward,
                                      cost_threshold)

        # Adjust sensor limits:
        self.sensor_limits = limits
