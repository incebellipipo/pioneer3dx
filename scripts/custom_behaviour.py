#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import time
from helpers import *
from robot import Robot


def main():
    rospy.init_node('custom_behaviour', anonymous=False, disable_signals=True)

    robot_name = ""
    robot = Robot(robot_name)
    robot.rate.sleep()
    points = [(0,0),(1,1),(2,2)]
    while not rospy.is_shutdown():
        for point in points:
            robot.go_to(point[0],point[1],drive=True)
        robot.rate.sleep()

if __name__ == "__main__":
    main()
