#!/usr/bin/env python

from state_manager.state_client import StateClient
from rospy import loginfo
import rospy


if __name__ == '__main__':

    rospy.init_node('test_client')
    client = StateClient()
    client.client_initialize()

    if client.change_state_and_wait(2):
        loginfo('After change to 2')
    if client.change_state_and_wait(4):
        loginfo('After change to 4')
    if client.change_state_and_wait(0):
        loginfo('After change to 0')

    client.transition_to_state(3)
    loginfo('After change to 3')
    client.transition_to_state(5)
    loginfo('After change to 5')
    client.transition_to_state(6)
    loginfo('After change to 6')
