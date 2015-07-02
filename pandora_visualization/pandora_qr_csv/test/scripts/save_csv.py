#!/usr/bin/env python

from __future__ import print_function
from rospy import ServiceProxy, ServiceException, wait_for_service

from pandora_qr_csv.srv import createCSV
from std_msgs.msg import String


class SaveMissionClient():
    def init():
        pass

    def save_mission_client(self, mission_name):
        print('WAITING FOR SEVICE')
        wait_for_service('qr_csv/createCSV')
        msg = String(mission_name)
        try:
            save_mission = ServiceProxy('qr_csv/createCSV', createCSV)
            save_mission(msg)
        except ServiceException as e:
            print('Service call failed: ' + str(e))

        print('Service call succeded')
        return True

if __name__ == "__main__":
    print('Requesting save Mission.')

    SMC = SaveMissionClient()
    SMC.save_mission_client("Testing")
