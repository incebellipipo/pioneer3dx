#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from robot import Robot
from potential_fields import *


def main():

    rospy.init_node('pf_optimization', anonymous=True, disable_signals=True)
    robot_name = rospy.get_param("~robot_name")

    robot = Robot(robot_name)

    robot.rate.sleep()

    obs_x, obs_y = None, None
    # Fixed definitions
    obs_ptype   = PointType.REPULSIVE
    goal_ptype  = PointType.ATTRACTIVE
    goal_radius = 0.2

    while True:
        if robot.pose_data is not None:
            break
        robot.rate.sleep()

    desired_x, desired_y = 10, 10 # todo argparser
    robot.initial_pose = (robot.pose_data[0], robot.pose_data[1])

    while not rospy.is_shutdown():

        if robot.obs_distance is not None:
            obs_x, obs_y = get_real_point_pose(
                robot.pose_data[0],
                robot.pose_data[1],
                robot.pose_data[2],
                robot.obs_distance,
                robot.obs_angle
            )
        if obs_x is None or obs_y is None:
            continue

        obs_point = Point(
                (obs_x, obs_y),
                0.15,
                2.0,
                PointType.REPULSIVE,
                5.0
                )

        goal_point = Point(
                (desired_x, desired_y),
                goal_radius,
                2.0,
                PointType.ATTRACTIVE,
                3.0
                )

        obs_vector = obs_point.get_vector((robot.pose_data[0], robot.pose_data[1]))
        goal_vector = goal_point.get_vector((robot.pose_data[0], robot.pose_data[1]))

        final_vector_x = goal_vector[0] + obs_vector[0]
        final_vector_y = goal_vector[1] + obs_vector[1]
        linear, angular = robot.go_to(final_vector_x, final_vector_y, drive=False)
        dist_to_obj, _ = calculate_difference((robot.pose_data[0], robot.pose_data[1]), (desired_x, desired_y))
        if dist_to_obj < 0.3: # 30 cm
            linear, angular = 0, 0
            robot.stop()

        robot.set_speed(linear, angular)
        robot.rate.sleep()


if __name__ == "__main__":
    main()