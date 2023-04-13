""" LiDAR SIM is intended to test methods and parameters for the eventual use
on a robotic mower. The intention is to better understand and experiment with
methods before implementation, hopefully allowing issues to be found and
visualised.

The methods are primarily based on the work found in:

Peng, Y., Qu, D., Zhong, Y., Xie, S., Luo, J., & Gu, J. (2015, August). The obstacle detection and obstacle avoidance algorithm based on 2-d lidar. In 2015 IEEE international conference on information and automation (pp. 1648-1653). IEEE.
"""

import cProfile
import csv
import glob
import os
import pstats
import random
import signal
import sys
import warnings

import geopandas as gpd
import numpy as np
import rclpy
from rclpy.node import Node
from shapely.geometry import LineString, Point, Polygon
from std_msgs.msg import String

from consts import *
from mower import Robot
from utm_func import to_utm, utm_dist

lidar_range = 30  # Number of LiDAR points (lasers) to generate
lidar_dist = 1  # Distance of LiDAR in front of robot in metres
move_dist = 0.3  # Distance to move per turn in metres
lidar_width = 15  # The angle from the centre to the left and right most LiDAR points
obj_gap = 0.5  # Minimum distance before two objects are considered seperate
try_count = 5  # Maximum number of movement tries before the robot may be considered stuck
Q_SIZE = 10  # Queue used for off course determining
MAX_SPEED = 0.6
OFF_COURSE_TARGET = -1


class NoTurns(Exception):
    pass


class NoPath(Exception):
    pass


def collinear(p0, p1, p2):
    """Determine if three points are collinear using the slope method
    """
    x1, y1 = p1[0] - p0[0], p1[1] - p0[1]
    x2, y2 = p2[0] - p0[0], p2[1] - p0[1]
    return abs(x1 * y2 - x2 * y1) < sys.float_info.epsilon


def get_line(a, b):
    if (b.x - a[0]) == 0:
        a[0] += np.finfo(np.float64).min
    m = (b.y - a[1]) / (b.x - a[0])
    c = a[1] - (m * a[0])
    return m, c


def perpen_point(p, a, b):
    """Get a point on a line perpendicular to a 3rd point.

    This function is used to determine the back-on-track point when
    the robot is off-course and not avoiding obstacles.

    Args:
        p: The given point - the robot's location
        a: The start point of the line
        b: The end point of the line
    """
    k = ((b[1] - a[1]) * (p[0] - a[0]) - (b[0] - a[0]) * (p[1] - a[1])) / (pow(
        (b[1] - a[1]), 2) + pow((b[0] - a[0]), 2))
    x4 = p[0] - k * (b[1] - a[1])
    y4 = p[1] + k * (b[0] - a[0])
    return np.array([x4, y4])


def avoidance(mower, path, target, nogos, centre_line, test_shape, current,
              right_bear, left_bear, centre_bear):
    """

    The main driver function to avoid obstacles.

    Args:
        mower: The lawn mower object.
        path: The traversal route.
        target: The current target within the route.
        nogos: The list of nogo-zones in the scene.
        centre_line: The centre line of the robot's LiDAR.
        test_shape: The outer perimeter of the scene.
        current: The current (previous target) within the route.
        img: The image number used for exporting images.
        right_bear: The list of bearings to try when turning right.
        left_bear: The list of bearings to try when turning left.
        centre_bear: The list of bearings to try when iteratively
            turning left and right.
    """
    while utm_dist([mower.x, mower.y], path[target]) > 0.2:
        m = 0
        detected_line, img = mower.get_detected_line(target, nogos, 0,
                                                     centre_line, test_shape,
                                                     current, img, path)
        # If detected objects iteratively look for new direction until none
        if detected_line is not None:

            centre_lines, left_lines, right_lines = mower.seperate_lidar_ranges(
            )
            lidar_intersect, side = mower.lidar_intersecting(
                centre_lines, left_lines, right_lines, detected_line)
            while lidar_intersect:
                if side == OBJECT_TO_LEFT:
                    bear = right_bear
                    line = right_lines
                elif side == OBJECT_TO_RIGHT:
                    bear = left_bear
                    line = left_lines
                else:
                    bear = centre_bear
                    line = centre_lines

                detected_line, img = mower.get_detected_line(
                    target, nogos, bear[m], line, test_shape, current, img,
                    path)

                # Once no obstacles are found, break to move foward
                if detected_line is None:
                    break
                centre_lines, left_lines, right_lines = mower.seperate_lidar_ranges(
                )
                lidar_intersect, side = mower.lidar_intersecting(
                    centre_lines, left_lines, right_lines, detected_line)
                # print(bear[m])
                m += 1
                # If no turning options left, end. - Handle points impossible to reach
                if m == len(bear) - 1:
                    print(side)
                    raise NoTurns("Couldn't get to point - no where to turn")

        mower.move(move_dist, path[target])
        if target == OFF_COURSE_TARGET:
            t = 1
        else:
            t = target
        mower.enqueue(mower.is_outside_buffer(path, current, t))
        mower.visited = np.vstack((mower.visited, [mower.x, mower.y]))
        m = 0
        # If the mower is off course return to pick new point on line
        if mower.is_off_course():
            return img
        # If the mower hasn't reached the point in the estimated time
        if mower.tries > mower.max_tries(path, target):
            return img
        # If mower is no longer off course
        if not mower.is_off_course() and target == -1:
            return img

    return img


def main(args=None):

    # Load objects and perimeter in Lat, Long format
    test_shape = np.loadtxt("./perimeter.out", dtype=float, delimiter=",")
    test_shape = to_utm(test_shape)
    path = np.loadtxt("./route.out", dtype=float, delimiter=",")
    path_len = len(path)
    current = 0  # int(path_len / 2) + 125
    skipped = 0
    target = current + 1

    # Init robot
    mower = Robot(path[current, 0], path[current, 1], target, Q_SIZE,
                  lidar_range, lidar_width, lidar_dist, move_dist, try_count,
                  obj_gap, REAL_TIME, DIRECT)
    mower.per_poly = Polygon(test_shape)
    mower.visited = np.vstack((mower.visited, [path[0, 0], path[0, 1]]))
    mower.lidar_lines()

    # If the robot detects an object to the right, turn left
    right_bear = list(range(4, 180, 4))
    # If the robot detects an object to the left, turn right
    left_bear = list(range(-4, -180, -4))
    # If the robot detects an object in front, try left and right
    centre_bear = [
        val for pair in zip(list(range(8, 180, 6)), list(range(-8, -180, -8)))
        for val in pair
    ]
    img = 0
    prev_apath = []
    # signal.signal(signal.SIGINT, signal_handler)

    while target < path_len and SIGINT == 0:
        # If reached target within region of inaccuracy
        if utm_dist([mower.x, mower.y], path[target]) <= 0.2:
            current += 1
            target += 1
            mower.tries = 0
            continue
        # If the mower is considered stuck
        if mower.tries > mower.max_tries(path, target):
            print("Trying A*")
            access, apath = mower.is_accessible(path[target])
            # If no path then skip this point as it is not reachable
            print(apath)
            if not access or apath == [[mower.x, mower.y]
                                       ] or apath == prev_apath:
                current += 1
                skipped += 1
                target += 1
                print("Path not possible, skipping checkpoint")
            else:
                print("Path may be possible, trying route")
                atarget = 1
                acurrent = 0
                for a in apath[:-1]:
                    # A* paths are not always possible given it relies on the
                    # robot's knowledge of the scene
                    try:
                        img = avoidance(mower, np.array(apath), atarget, nogos,
                                        centre_line, test_shape, acurrent, img,
                                        right_bear, left_bear, centre_bear)
                        print("Reached A* target " + str(acurrent) +
                              " out of " + str(len(apath)))
                    except (NoTurns, NoPath) as e:
                        print(e)
                        print("Couldn't reach A* target" + str(atarget))
                        break
                    atarget += 1
                    acurrent += 1
            mower.tries = 0
            prev_apath = apath
        # If the mower is off course
        elif mower.is_off_course():
            # Pick a point between robot and desired course
            p = perpen_point([mower.x, mower.y], path[current], path[target])
            if utm_dist([mower.x, mower.y], p) <= 0.2 or utm_dist(
                [mower.x, mower.y], path[target]) <= 0.2:
                # Would require more confirmation points in real application
                # to determine if back on course
                mower.clear_q()
                continue
            dx = p[0] - path[target][0]
            dy = p[1] - path[target][1]
            # Offset this point to be towards the next route 'checkpoint'
            if not REAL_TIME:
                p = np.array([p[0] + (-0.1) * dx, p[1] + (-0.1) * dy])
            # Check if object is between mower and desired route
            detected_line, img = mower.get_detected_line(
                OFF_COURSE_TARGET, nogos, 0, centre_line, test_shape, current,
                img, np.array([path[current], path[target], p]))
            # If mower is avoiding obstacles - acceptable to be off course
            if detected_line is not None:
                mower.clear_q()
                continue
            try:
                img = avoidance(mower,
                                np.array([path[current], path[target],
                                          p]), OFF_COURSE_TARGET, nogos,
                                centre_line, test_shape, 0, img, right_bear,
                                left_bear, centre_bear)
            except (NoTurns, NoPath) as e:
                current += 1
                target += 1
                skipped += 1
                mower.tries = 0
                print(e)
                print("Couldn't reach target")
        else:
            try:
                img = avoidance(mower, path, target, nogos, centre_line,
                                test_shape, current, img, right_bear,
                                left_bear, centre_bear)
            except (NoTurns, NoPath) as e:
                current += 1
                target += 1
                skipped += 1
                mower.tries = 0
                print(e)
                print("Couldn't reach target")

    print("Got to point " + str(current) + " out of " + str(path_len) +
          " had to skip " + str(skipped))
    print("Total area: " + str(Polygon(test_shape).area) + " m^2")
    print("Total area covered: " + str(s.buffer(0.15)[0].area) + " m^2")
    missed = 0
    for n in mower.nogo_polys:
        missed += n.area
    print("Total area inside no-go: " + str(missed) + " m^2")
    print("Total distance travelled: " + str(mower.dist_travelled) + " m")


if __name__ == "__main__":
    main()
