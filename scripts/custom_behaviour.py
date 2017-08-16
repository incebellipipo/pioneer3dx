#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from robot import Robot
from geometry_msgs.msg import Pose2D


currentGoal = None


def poseCallback(msg):
    global currentGoal
    currentGoal = msg

def main():
    rospy.init_node('custom_behaviour', anonymous=False, disable_signals=True)
    rospy.Subscriber("/kovan/pose", Pose2D, poseCallback, queue_size=1)
    global currentGoal
    robot_name = ""
    robot = Robot(robot_name)
    robot.rate.sleep()
    while not rospy.is_shutdown():
        if currentGoal is not None:
            linear, angular = robot.go_to(currentGoal.x, currentGoal.y, drive=False)
            robot.set_speed(linear, angular)

        robot.rate.sleep()

if __name__ == "__main__":
    main()
