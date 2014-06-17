#! /usr/bin/env python

import roslib
import rospy


import actionlib
from pandora_rqt_gui.msg import ValidateVictimGUIAction, ValidateVictimGUIGoal


def victim_validation_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient("victimValidation", ValidateVictimGUIAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = ValidateVictimGUIGoal()
    goal.victimFoundx = 3.2
    goal.victimFoundy = 6.3
    goal.probability = 0.8
    goal.sensorIDsFound = "Eleana for the Win !"

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('victim_validation_client_py')
        result = victim_validation_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
