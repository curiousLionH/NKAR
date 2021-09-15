#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import threading
import time
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovariance
import control as CAR


class MissionManager:
    def __init__(self):
        self.missions = dict()
        self.missions2 = dict()
        self.mission_keys = list()
        self.mission_idx = None
        self.current_mission_key = None
        self.manager_thread = threading.Thread(target=self.main)
        self.mission_manager = None




    def add_mission(self, key, mission):
        # if key not in self.mission_keys:
        #     warnings.warn("The new key %s is not registered.\
        #          Therefore, NO mission will be added." % key)
        # else:
        mission.key = key
        self.missions[key] = mission

    def main(self):
        self.next_mission()

    def next_mission(self):
        self.mission_idx = 1    # 미션인덱스를 subscribe해서 넣어주어야함
        self.current_mission_key = self.mission_keys[self.mission_idx]
        return self.current_mission_key

    def start(self):
        self.manager_thread.start()

    def join(self):
        self.manager_thread.join()


class Mission(object):
    def __init__(self, control, path):
        self.control = control
        self.path = path
        self.key = None

    def main(self):  # 미션 수행 함수 구현해야함.
        '''
        while not self.mission_end():
            (params) = self.line.some_func_for_specific_mission()
            self.control.some_func_for_specific_mission(*params)
            time.sleep(0.1)
        '''
        pass

    def __str__(self):
        if self.key is None:
            return "None"
        else:
            return self.key + " Mission"


class GpsTrackingMission(Mission):
    def __init__(self, logging_data_idx, db, control, path):
        super(GpsTrackingMission, self).__init__(db, control, path)

        self.speed = 100
        self.current_position = []
        rospy.Subscriber("/rtabmap/localization_pose",PoseWithCovariance,self.pose_cb)


    def main(self):
        start_time = time.time()

        self.control.gps_mission_end = False

        # filename = self.logging_filename_dict[self.logging_data_idx]
        # filepath = os.path.join('/home/heven/catkin_ws/src/HEVEN-AutonomousCar-2021/scripts', 'src', 'Database', filename)

        while True:
            print("start gps tracking")
            distance = self.distance()
            if distance < 100:
                self.control.gps_mission_end = True
            if self.control.gps_mission_end:
                # 미션 탈출
                break

            self.steer =  # TEB local planner가 보내준 제어값을 콘트롤값에 넣어줌
            CAR.move()
            time.sleep(0.1)


    def distance(self):
        self.dest = self.missions[current_mission_key]
        distance = ((self.dest[0] - self.current_position[0])^2 + (self.dest[1] - self.current_position[1])^2)
        return distance

    def pose_cb(self, data):
        self.current_position[0] = data.x
        self.current_position[1] = data.y




class EndMission(Mission):
    def main(self):
        while True:
            self.control.stop()


if __name__ == "__main__":
    # db = Database(gps=False, imu=False, platform=False, cam=False, lidar=False)
    # db.start()

    time.sleep(1)


    # obstacle = StaticSmallObstacleMission(db=db, control=control, path=path)
    # left_traffic = LeftTrafficLightMission(db=db, control=control, path=path)
    # straight_traffic = StraightTrafficLightMission(db=db, control=control, path=path)
    # gps_tracking = GpsTrackingMission(db=db, control=control, path=path)
    # default = DefaultMission(db=db, control=control, path=path)
    # parking = ParkingMission(db=db, control=control, path=path)
    #
    # obstacle.main()
    # left_traffic.main()
    # straight_traffic.main()
    # gps_tracking.main()
    # default.main()
    # parking.main()
