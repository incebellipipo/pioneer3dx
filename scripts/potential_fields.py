# -*- coding: utf-8 -*-

"""
Potential fields related classes
"""

import math
from enum import Enum
from helpers import calculate_difference, sign


def get_real_point_pose(r_x, r_y, r_t, obs_dist, obs_angle):
    """
    Get real point from vales gathered from actual robot
    :param r_x: robot pose x
    :param r_y: robot pose y
    :param r_t: robot pose theta in radians
    :param obs_dist: distance to obstacle
    :param obs_angle: angle to obstacle in degrees
    :return: tuple (obs_x, obs_y)
    """
    obs_x = r_x - obs_dist * math.cos(r_t + math.radians(obs_angle))
    obs_y = r_y - obs_dist * math.sin(r_t + math.radians(obs_angle))
    return obs_x, obs_y


class PointType(Enum):
    """
    Point type enumeration
    """
    ATTRACTIVE, REPULSIVE = xrange(2)


class Point(object):
    """
    Potential fields point
    """

    def __init__(self):
        pass

    def __init__(self, pose, radius, spread, ptype, alpha_beta):
        self.pose = pose
        self.radius = radius
        self.spread = spread
        self.ptype = ptype

        # attraction and repulsion
        self.alpha_beta = alpha_beta

    def get_vector(self, pose):
        """
        Finds the vector to given position
        :param pose: Position tuple
        :return: Vector tuple
        """
        distance, angle = calculate_difference(pose, self.pose)
        delta_x = delta_y = 0.0

        if self.ptype is PointType.ATTRACTIVE:
            if distance < self.radius:
                delta_x = delta_y = 0.0
            elif self.radius <= distance <= self.radius + self.spread:
                value = self.alpha_beta * (distance - self.radius)
                delta_x = value * math.cos(angle)
                delta_y = value * math.sin(angle)
            elif distance > self.radius + self.spread:
                value = self.alpha_beta * self.spread
                delta_x = value * math.cos(angle)
                delta_y = value * math.sin(angle)

        else:
            if distance < self.radius:
                delta_x = -sign(math.cos(angle))
                delta_y = -sign(math.sin(angle))
            elif self.radius <= distance <= self.radius + self.spread:
                value = -self.alpha_beta * (self.spread + self.radius - distance)
                delta_x = value * math.cos(angle)
                delta_y = value * math.sin(angle)
            elif distance > self.radius + self.spread:
                delta_x = delta_y = 0.0

        return delta_x, delta_y