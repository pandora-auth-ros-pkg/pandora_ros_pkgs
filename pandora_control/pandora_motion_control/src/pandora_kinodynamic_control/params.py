NAVIGATION_TOPIC = "/cmd_vel"
ACTUAL_TRAJECTORY_TOPIC = "/robot_trajectory"
COMMAND_TOPIC = "/kinematic_parameters"

WORLD = "/world"
BASE_LINK = "/base_link"

# Reinforcement Learning Related:
# 1) States:
# i) Number of States
ROLL    = 7
PITCH   = 7
LINEAR  = 7
ANGULAR = 7

STATES = [ROLL,PITCH,LINEAR,ANGULAR]

# ii) Limits of each state [format = (low,high)]
ROLL_LIMITS    = (-0.28,0.28)  # input in rads
PITCH_LIMITS   = (-0.28,0.28)  # input in rads
LINEAR_LIMITS  = (-0.31,0.31)    # in m/s
ANGULAR_LIMITS = (-0.61,0.61)    # in rad/s

LIMITS = [ROLL_LIMITS,PITCH_LIMITS,LINEAR_LIMITS,ANGULAR_LIMITS]

# 2) Actions:
# i) Number of Actions
ACTION_STATES = 8

# ii) Action ranges
ACTION_RANGE = (0.8,1.8)

# 3) Agent
alpha = 0.5
gamma = 0.1
epsilon = 0.4  # default value = 0.29

# 4) Cost Function :
MAX_REWARD = 2
COST_THRESHOLD = 1

# 5) General:
FUSION_WEIGHTS = [1,0.0]
TIME_GRANULARITY = 5
COMMAND_DURATION = 0.2
STEP_SIZE = 1       # cmd_vel callbacks ,until agent learn
VISUALIZATION = False

# 6) Store Results:
FILENAME = "AV_table_"
SAVE_STEP_SIZE  = 50
