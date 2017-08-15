#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math

from scipy.signal import argrelmin


def calculate_difference(point1, point2):
    """
    Calculates difference between two points.
    Returns distance and angle (degrees)

    :param point1: Position tuple for point 1
    :param point2: Position tuple for point 2
    :return: distance and angle (degrees)
    """
    x_diff = point2[0] - point1[0]
    y_diff = point2[1] - point1[1]

    distance = math.sqrt(x_diff ** 2 + y_diff ** 2)
    angle = math.atan2(y_diff, x_diff)

    return distance, angle

def find_minima(scan_data):
    """
    Finds minima of the scan data
    using wrap mode

    :param scan_data: Laser scan data
    :return: Minimum points of scan data
    """
    return argrelmin(scan_data, mode='wrap')[0]

def get_laser_angle(point):
    """
    Calculates laser angle for given point

    :param point: Laser point number
    :return: Angle in radians
    """
    return (2 * math.pi * (point + 1) / 360) - math.pi

def sign(x):
    return math.copysign(1, x)