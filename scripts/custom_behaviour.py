#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import time
from helpers import *
from robot import Robot


def main():
    rospy.init_node('custom_behaviour', anonymous=False, disable_signals=True)

    robot_name = rospy.get_param('~robot_name')
    # robot_name = "pioneer3dx"
    time.sleep(10)

    robot = Robot(robot_name)
    robot.rate.sleep()

    desired_x, desired_y = 5.5, 8.8
    # desired_x, desired_y = 0.0, 0.0

    pose_before = robot.pose_data 
    diff_in_x, diff_in_y = 0, 0
    total_distance_traveled = 0

    
    while not rospy.is_shutdown():
        distance, _ = calculate_difference(pose_before,robot.pose_data)        
        total_distance_traveled += distance
        if robot.nav_goal is not None:
            desired_x, desired_y = robot.nav_goal[0], robot.nav_goal[1]

        linear, angular = robot.go_to(desired_x, desired_y, drive=False)
        # end flag
        pose_before = robot.pose_data

        robot.set_speed(linear, angular)
        robot.rate.sleep()

if __name__ == "__main__":
    main()
