#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from robot import Robot


def main():
    rospy.init_node('custom_behaviour', anonymous=False, disable_signals=True)

    # robot_name = rospy.get_param('~robot_name')
    robot_name = "pioneer3dx"

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    robot = Robot(robot_name)
    robot.rate.sleep()

    desired_x, desired_y = 5.0, 8.0

    while not rospy.is_shutdown():
        linear, angular = robot.go_to(desired_x, desired_y, drive=False)


        robot.set_speed(linear, angular)
        robot.rate.sleep()

if __name__ == "__main__":
    main()