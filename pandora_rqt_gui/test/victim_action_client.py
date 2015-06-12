#! /usr/bin/env python

from __future__ import print_function
import rospy
import random

import actionlib
from pandora_gui_msgs.msg import ValidateVictimGUIAction, ValidateVictimGUIGoal

validate_victim_service_name = '/gui/validate_victim'


def victim_validation_client():
    # Creates the SimpleActionClient, passing the type of the action

    client = actionlib.SimpleActionClient(validate_victim_service_name,
                                          ValidateVictimGUIAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Create a random goal to send to the action server.
    sensors = ['motion', 'co2', 'thermal', 'sound', 'victim', 'sound']
    random.shuffle(sensors)
    goal = ValidateVictimGUIGoal()
    goal.victimFoundx = random.random() * 20
    goal.victimFoundy = random.random() * 20
    goal.probability = random.random() * 1
    goal.sensorIDsFound = sensors[0:3]

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('victim_validation_client_py')
        result = victim_validation_client()
        print(result)
    except rospy.ROSInterruptException:
        print('Program interrupted before completion.')
