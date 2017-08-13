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

    while not rospy.is_shutdown():
        robot.go_to(-10,-20,drive=True)
        robot.rate.sleep()

if __name__ == "__main__":
    main()
