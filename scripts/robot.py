#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker

import tf
import cv2
import cv_bridge
import math
import time
from helpers import calculate_difference, find_minima, get_laser_angle


class Robot(object):
    def __init__(self, name = ""):

        self.name = name

        # Publisher
        self.cmd_vel = rospy.Publisher( self.name + "/cmd_vel",
                                       Twist, queue_size=10)

        # Subscriber
        self.odom = rospy.Subscriber( self.name + "/odom",
                                     Odometry, self.odom_callback,
                                     queue_size=1)

        self.laser = rospy.Subscriber( self.name + "/front_laser/scan",
                                     LaserScan, self.laser_callback,
                                      queue_size=1)

        self.camera = rospy.Subscriber( self.name + "/front_camera/image_raw",
                                       Image, self.camera_callback,
                                       queue_size=1)

        self.marker = rospy.Subscriber( self.name + "/marker",
                                       Marker, self.marker_callback,
                                       queue_size=1)

        self.nav_goal = rospy.Subscriber("/move_base_simple/goal",
                                       PoseStamped, self.nav_goal_callback,
                                         queue_size=1)

        self.pose_data = None
        self.laser_data = None
        self.camera_data = None
        self.marker_data = None
        self.nav_goal = None
        self.obs_distance = None
        self.obs_angle = None
        self.cv_bridge = cv_bridge.CvBridge()

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
        if msg is None: return
        self.laser_data = np.array(msg.ranges)
        minima = find_minima(self.laser_data)
        if len(minima) is 0: return
        min_angle = get_laser_angle(minima[0])
        min_distance = self.laser_data[min_angle]

        for i in minima:
            hold_distance = self.laser_data[i]
            if hold_distance < min_distance:
                min_distance = hold_distance
                min_angle = i

        self.obs_distance = min_distance
        self.obs_angle = min_angle

    def camera_callback(self, msg):
        try:
            self.camera_data = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError:
            return

        gray = cv2.cvtColor(self.camera_data, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 30, 150)

        # cv2.imshow("Robot Camera", canny)
        cv2.waitKey(1)

    def marker_callback(self, msg):
        self.marker_data = msg

    def nav_goal_callback(self, msg):
        self.nav_goal = [msg.pose.position.x, msg.pose.position.y]

    def set_speed(self, linear, angular):
        movecmd = Twist()
        movecmd.linear.x = linear
        movecmd.angular.z = angular
        self.cmd_vel.publish(movecmd)

    def stop(self):
        self.set_speed(0.0, 0.0)

    def go_to(self, x, y, drive=False):
        distance, angle = calculate_difference((self.pose_data[0], self.pose_data[1]), (x, y))
        while distance is not 0.0:
            distance, angle = calculate_difference((self.pose_data[0], self.pose_data[1]), (x, y))
            error_angle = angle - self.pose_data[2]
            error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
            angular = error_angle * 1.0
            distance = distance if distance > 0.1 else 0.0
            linear = 0.4 if distance > 0.5 else distance * 0.5
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
