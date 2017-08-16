#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import tf
import math
from helpers import calculate_difference, find_minima


class Robot(object):
    def __init__(self, name = ""):

        self.name = name

        # Publisher
        self.cmd_vel = rospy.Publisher( self.name + "/cmd_vel_mux/input/teleop",
                                       Twist, queue_size=10)

        # Subscriber
        self.odom = rospy.Subscriber( self.name + "/odom",
                                     Odometry, self.odom_callback,
                                     queue_size=1)

        self.laser = rospy.Subscriber( self.name + "/scan",
                                     LaserScan, self.laser_callback,
                                      queue_size=1)

        self.pose_data = None
        self.laser_data = None
        self.rate = rospy.Rate(10)
        self.rate.sleep()

    def odom_callback(self, msg):
        if msg is None: return
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        quat = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        euler = tf.transformations.euler_from_quaternion(quat)

        self.pose_data = [
            position.x,
            position.y,
            euler[2]
        ]

    def laser_callback(self, msg):
        """
        Updates laser_data variable to
        data coming from front laser

        :param msg: LaserScan callback data
        """
        if msg is None:
            return
        self.laser_data = np.array(msg.ranges)
        minima = find_minima(self.laser_data)
        if len(minima) is 0:
            return

    def set_speed(self, linear, angular):
        movecmd = Twist()
        movecmd.linear.x = linear
        movecmd.angular.z = angular
        self.cmd_vel.publish(movecmd)

    def stop(self):
        self.set_speed(0.0, 0.0)

    def go_to(self, x, y, drive=True, with_pose=True):
        distance, angle = calculate_difference((self.pose_data[0], self.pose_data[1]), (x, y))
        while distance is not 0.0:
            distance, angle = calculate_difference((self.pose_data[0], self.pose_data[1]), (x, y))
            if with_pose is False:
                distance, angle = calculate_difference((0, 0), (x, y))
            error_angle = angle - self.pose_data[2]
            error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
            angular = error_angle * 1.0 # PID with just P
            distance = distance if distance > 0.1 else 0.0
            linear = 0.5 if distance > 0.5 else distance * 0.5
            if not drive:
                break
            self.set_speed(linear, angular)
            self.rate.sleep()
        return linear, angular


def main():
    rospy.init_node("robot_controller")
    robot_name = rospy.get_param("~robot_name")
    r = Robot(robot_name)

    while True:
        r.rate.sleep()
        if rospy.is_shutdown():
            break

if __name__ == '__main__':
    main()
