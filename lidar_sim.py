############### TODO ##################
# Need to determine if point is inaccesbile

########### NOTE ############
# In other scripts such as 'coverage' perimeter and
# nogo zones are handled seperately. In this the
# perimeter is considered solid. This is for two reasons.
# Firstly, if the perimeter is physical then it would
# be considered an obstacle. Secondly, if the perimeter
# is not solid then the RTK methods would prevent the
# robot going off course.

import cProfile
import csv
import glob
import itertools
import math
import os
import pstats
import random
import signal
import sys
import time
import warnings
from inspect import getmembers, isfunction

import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
import utm
from mathutils import Vector
from mathutils.geometry import intersect_point_line
from matplotlib.colors import ListedColormap
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.best_first import BestFirst
from pathfinding.finder.bi_a_star import BiAStarFinder
from pathfinding.finder.breadth_first import BreadthFirstFinder
from pathfinding.finder.dijkstra import DijkstraFinder
from pathfinding.finder.msp import MinimumSpanningTree
from shapely.geometry import LineString, Point, Polygon

from surveytoolbox.bdc import bearing_distance_from_coordinates
from surveytoolbox.cbd import coordinates_from_bearing_distance
from surveytoolbox.config import BEARING, EASTING, ELEVATION, NORTHING

lidar_range = 60  # Number of LiDAR points (lasers) to generate
lidar_dist = 1  # Distance of LiDAR in front of robot in metres
move_dist = 0.3  # Distance to move per turn in metres
lidar_width = 15  # The angle from the centre to the left and right most LiDAR points
obj_gap = 0.3  # Minimum distance before two objects are considered seperate

ON_COURSE = 0
OFF_COURSE = 1
Q_SIZE = 10
MAX_SPEED = 0.6
PRINT = False
OFF_COURSE_TARGET = -1
OBJECT_TO_LEFT = -1
OBJECT_TO_RIGHT = 1
OBJECT_TO_CENTRE = 0
SIGINT = 0
DIRECT = False
REAL_TIME = False


class NoTurns(Exception):
    pass


class NoPath(Exception):
    pass


class Ends:

    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def line(self):
        return LineString([self.p1, self.p2])

    def array(self):
        return np.array([[self.p1.x, self.p1.y], [self.p2.x, self.p2.y]])

    def points(self):
        return [self.p1, self.p2]

    def close_to(self, other):
        if utm_dist(self.p1, other.p1) < 0.5:
            return True
        if utm_dist(self.p1, other.p2) < 0.5:
            return True
        if utm_dist(self.p2, other.p1) < 0.5:
            return True
        if utm_dist(self.p2, other.p2) < 0.5:
            return True
        return False

    def overlaps(self, other):
        for r in itertools.product(self.points(), other.points()):
            if r[0].touches(r[1]):
                return True
        return False

    def touches(self, other):
        if self.line().intersects(other.line()):
            return True
        return False

    def collinear(self, other):
        s1 = (self.p2.y - self.p1.y) / (self.p2.x - self.p1.x)
        s2 = (other.p2.y - other.p1.y) / (other.p2.x - other.p1.x)
        return math.degrees(math.atan((s2 - s1) / (1 + (s2 * s1)))) < 5

    def is_same_line(self, other):
        if self.close_to(other) or self.overlaps(other) or self.touches(other):
            return True
        return False


class Robot:
    """The Robot class is the simulated version of a lawn mower
    used for testing obstacle detection, mapping, and
    area coverage.
    """

    def __init__(self, x, y, target):
        self.x = x
        self.y = y
        self.visited = np.empty((0, 2))
        self.heading = float()
        self.target = target
        self.inside = [0] * Q_SIZE
        self.nogo_polys = []
        self.per_poly = None
        self.dist_travelled = 0
        self.detected_ends = []
        self.detected_points = []
        self.tries = 0

    # Route traversal functions start
    def enqueue(self, val):
        """Keep track of the n amount of on-course and off-course
        during traversal

        This queue is used to keep a n amount of on and off course
        readings to confirm whether the robot has become off course.
        Only one value may be an error and so a number are used to
        confirm.

        Args:
            val: Whether the robot was on-course or off-course
        """
        for i in range(len(self.inside) - 1):
            self.inside[i + 1] = self.inside[i + 1] ^ self.inside[i]
            self.inside[i] = self.inside[i + 1] ^ self.inside[i]
            self.inside[i + 1] = self.inside[i + 1] ^ self.inside[i]
        self.inside[len(self.inside) - 1] = val

    def per_on_course(self):
        """Determines the percentage of values in the queue to be
        on-course
        """
        res = len(self.inside)
        for i in self.inside:
            res -= i
        res /= len(self.inside)
        return res

    def is_off_course(self):
        """Based on the percentage of on-course values, decide if the
        robot is considered off-course or not
        """
        if self.per_on_course() < 0.8:
            return OFF_COURSE
        return ON_COURSE

    def clear_q(self):
        """Resets the queue. Used when the robot confirms reaching a
        checkpoint - may not be used in real application.
        """
        self.inside = [0] * Q_SIZE

    def seperate_lidar_ranges(self):
        centre_lines = self.lidar_range()[0][int(lidar_range *
                                                 (1 / 3)):int(lidar_range *
                                                              (2 / 3))]
        left_lines = self.lidar_range()[0][0:int(lidar_range * (1 / 3))]
        right_lines = self.lidar_range()[0][int(lidar_range * (2 / 3)):-1]
        return centre_lines, left_lines, right_lines

    def is_outside_buffer(self, path, current, target):
        """Based on the robots distance from the line between its
        origin and target whilst traversing a path, it is outside the
        region of inaccuracy (the buffer).

        Args:
            path:
            current:
            target:
        """
        p2 = path[target]
        p1 = path[current]
        p3 = [self.x, self.y]
        per_dist = np.linalg.norm(np.cross(p2 - p1,
                                           p1 - p3)) / np.linalg.norm(p2 - p1)
        if per_dist > 0.2:
            return OFF_COURSE
        return ON_COURSE

    # end

    def max_tries(self, path, target):
        """Determine if the robot has exceeded the maximum number of
        tries when traversing to a point

        If the robot has no progress in the last N moves then it may
        be stuck. This function with determine this, and in turn cause
        A* to be called to find a new path.
        """
        if len(self.visited) > 5 and self.tries > 5:
            dist = utm_dist([self.x, self.y], self.visited[-5])
            if dist <= move_dist:
                return 0
        # tries = int((dist / move_dist) * 1.1)
        # print("Try " + str(self.tries) + " out of " + str(tries))
        # if dist <= 1:
        #     tries = 0
        # else:
        #     tries = self.tries + 1
        return self.tries + 1

    def print_finder_graph(self, target, b, matrix):
        go = [
            int(shift_float(self.x) - shift_float(b[0])),
            int(shift_float(self.y) - shift_float(b[1]))
        ]
        to = [
            int(shift_float(target[0]) - shift_float(b[0])),
            int(shift_float(target[1]) - shift_float(b[1]))
        ]
        matrix[int(shift_float(self.x) - shift_float(b[0]))][int(
            shift_float(self.y) - shift_float(b[1]))] = 0
        matrix[int(shift_float(target[0]) - shift_float(b[0]))][int(
            shift_float(target[1]) - shift_float(b[1]))] = 0
        plt.text(to[0], to[1], 'target', ha='center', va='center')
        plt.text(go[0], go[1], 'current', ha='center', va='center')
        plt.imshow(matrix, interpolation=None, cmap='viridis')
        plt.show()

    def detected_points_to_lines(self):
        """Reduce the detected points to simpler linestrings.

        Using all detected points quickly gets out of hand, so lines
        are used instead. The end points of these lines can be used to
        replace the current detected points to save memory and
        processing time.
        """
        lines = []
        len_d = 0
        self.detected_points = list(set(self.detected_points))
        for i in self.detected_ends:
            self.detected_points.append(i.p1)
            self.detected_points.append(i.p2)
        while len_d < len(self.detected_points) - 1:
            line = [self.detected_points[len_d]]
            while utm_dist(
                [self.detected_points[len_d].x, self.detected_points[len_d].y],
                [
                    self.detected_points[len_d + 1].x,
                    self.detected_points[len_d + 1].y
                ]) < obj_gap and len_d < len(self.detected_points) - 2:
                line.append(list(self.detected_points)[len_d + 1])
                len_d += 1
            if len(line) >= 2:
                lines.append(LineString(line))
            len_d += 1
        return lines

    def reduce_detected_points(self):
        """Reduce all points on the same line down the only the end
        points.

        This is done to reduce memory and processing time. Inner
        points are still needed for the A* grid but that function
        shouldn't need to be run very often.
        """
        lines = self.detected_points_to_lines()
        for line in lines:
            self.detected_ends.append(
                Ends(Point([line.coords[0][0], line.coords[0][1]]),
                     Point([line.coords[-1][0], line.coords[-1][1]])))
        new_lines = []
        added_ends = set()
        for i in self.detected_ends:
            if i.p1 in added_ends or i.p2 in added_ends:
                continue
            merge_points = [i.p1, i.p2]
            for j in self.detected_ends:
                if i == j:
                    continue
                if i.is_same_line(j):
                    if j.p1 not in added_ends:
                        merge_points.append(j.p1)
                        added_ends.add(j.p1)
                    if j.p2 not in added_ends:
                        merge_points.append(j.p2)
                        added_ends.add(j.p2)
            if len(new_lines) > 2:
                new_lines.append(LineString(merge_points))

        new_lines = list(set(new_lines))
        for line in new_lines:
            p1 = Point([line.coords[0][0], line.coords[0][1]])
            p2 = Point([line.coords[-1][0], line.coords[-1][1]])
            self.detected_ends.append(Ends(p1, p2))

        # self.detected_points = []

    def clean_apath(self, path, b):
        """Takes an A* path and removes any points that violates a
        known boundary.
        """
        apath = []
        lines = self.detected_points_to_lines()
        for p in path:
            # Points need shifting back to original UTM scale
            apath.append([
                shift_int(p[0] + shift_float(b[0])),
                shift_int(p[1] + shift_float(b[1]))
            ])
        checked_apath = [[self.x, self.y]]
        # Check if the route violates the detected boundaries
        for p in apath:
            coll = False
            for pp in apath:
                if pp == p:
                    continue
                # If two points in apath cross a detected point
                line2 = LineString([p, pp])
                for line in lines:
                    if line.intersects(line2):
                        coll = True
                        break
            if not coll:
                checked_apath.append(p)
            else:
                break

        print(checked_apath)
        for i in range(2):
            if len(checked_apath) > 2:
                del checked_apath[2 - 1::2]
        return checked_apath

    def is_accessible(self, target):
        """When the robot is unable to make progress it may be stuck,
        using it's knowledge of the scene an A* path is generated. If
        no path can be generated then the point is unreachable and
        shall be skipped.
        """
        b = self.per_poly.bounds  # minx, miny, maxx, maxy
        d = list(set(self.detected_points_to_lines()))
        m_x = int(shift_float(b[2]) - shift_float(b[0])) + 1
        m_y = int(shift_float(b[3]) - shift_float(b[1])) + 1
        matrix = [[1] * max(m_x, m_y)] * max(m_y, m_x)
        matrix = np.array(matrix)
        print(len(d))

        for line in d:
            for point in line.coords:
                x = int(shift_float(point[0]) - shift_float(b[0]))
                y = int(shift_float(point[1]) - shift_float(b[1]))
                for i in [-1, 0, 1]:
                    for j in [-1, 0, 1]:
                        matrix[x + i][y + j] = 0

        # self.print_finder_graph(target, b, matrix)
        grid = Grid(matrix=matrix)
        start = grid.node(int(shift_float(self.x) - shift_float(b[0])),
                          int(shift_float(self.y) - shift_float(b[1])))
        end = grid.node(int(shift_float(target[0]) - shift_float(b[0])),
                        int(shift_float(target[1]) - shift_float(b[1])))

        # AStarFinder, BestFirst, BiAStarFinder, DijkstraFinder,
        # IDAStarFinder, BreadthFirstFinder, MinimumSpanningTree
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)

        apath = self.clean_apath(path, b)
        return len(apath) > 1, apath

    def lidar_range(self):
        """Generate an array of lines at different angles to simulate
        the robot's LiDAR

        Returns:
            inter in index 0 is the LiDAR array used for detection.
            The polygon in index 1 is simply used for the visual
            output.
        """
        # Get the left and right most lines for the visual output
        left = coordinates_from_bearing_distance(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, self.heading - lidar_width, lidar_dist)
        right = coordinates_from_bearing_distance(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, self.heading + lidar_width, lidar_dist)
        inter = []
        # Generate each line with given bearing and length
        for i in range(lidar_range):
            pos = coordinates_from_bearing_distance(
                {
                    EASTING: self.x,
                    NORTHING: self.y,
                    ELEVATION: 0
                }, (self.heading - lidar_width) + (i / 2), lidar_dist)

            inter.append(LineString([(self.x, self.y), (pos['e'], pos['n'])]))

        return [
            inter,
            Polygon([(self.x, self.y), (left['e'], left['n']),
                     (right['e'], right['n']), (self.x, self.y)])
        ]

    def find_nearest(self, points):
        """Given a set of detected objects, find the points nearest to
        the robot. The robot will handle the closest object first.

        Args:
            points: All detected points.
        
        Return:
            The nearest object, or merged objects.
        """
        nearest = points
        for i in range(1, len(points)):
            # If robot cannot fit in the gap consider it one solid object
            if utm_dist([points[i - 1].x, points[i - 1].y],
                        [points[i].x, points[i].y]) < obj_gap:
                nearest.append(points[i])
            else:
                # If the new point is closer make new list - tackle closest objects first
                if utm_dist(
                    [self.x, self.y], [points[i].x, points[i].y]) < utm_dist(
                        [self.x, self.y], [points[i - 1].x, points[i - 1].y]):
                    nearest = [points[i]]
        return nearest

    def remove_hidden(self, points):
        """Remove any detected points that would not be visible in a
        real application.

        A point is considered hidden if another point is collinear and
        closer to the robot.

        Args:
            points: All points detected by the robot that are in range
                and being considered at this time.
        """
        new_points = []
        for i in range(len(points)):
            behind = False
            line = LineString([(self.x, self.y), (points[i].x, points[i].y)])
            for j in range(len(points)):
                if i != j and line.distance(points[j]) < 1e-8:
                    if utm_dist([self.x, self.y],
                                [points[i].x, points[i].y]) > utm_dist(
                                    [self.x, self.y],
                                    [points[j].x, points[j].y]):
                        behind = True
                        break
            if not behind:
                new_points.append(points[i])

        return new_points

    def detect_objects(self, nogos, target):
        """Find all objects within range of the robot's LiDAR.
        
        This method does require the objects to be 'known' but will
        only return those visible to the robot.
        
        Args:
            nogos: The list of nogos in the scene.
            target: The robot's current coordinate target, typically
                the next in the given route.
        
        Returns:
            All objects within range of the robot.
        """
        points = []
        lidar_lines = self.lidar_range()[0]
        for nogo in nogos[0]:  # For each object
            for i in range(-1, len(nogo) - 1):  # Loop over each corner
                # Create a solid edge to the object
                edge = LineString([(nogo[i][0], nogo[i][1]),
                                   (nogo[i + 1][0], nogo[i + 1][1])])
                for line in lidar_lines:
                    if line.intersects(edge):
                        # If the target is infront of the end don't consider it
                        if utm_dist([self.x, self.y], [
                                line.intersection(edge).x,
                                line.intersection(edge).y,
                        ]) > utm_dist([self.x, self.y], target):
                            continue
                        # If the distance is far away don't consider
                        if utm_dist([self.x, self.y], [
                                line.intersection(edge).x,
                                line.intersection(edge).y,
                        ]) > lidar_dist:
                            continue
                        points.append(line.intersection(edge))

        # Once all relevant points are found, determine which to handle first
        if len(points) > 0:
            points = self.remove_hidden(points)
            for p in points:
                self.detected_points.append(p)
            points = self.find_nearest(points)
            # self.reduce_detected_points()

        return points

    def closer_point(self, p1, p2):
        """Finds the closer of two points on a UTM grid
        
        Returns:
            The closest point.
        """
        if utm_dist([self.x, self.y], [p1.x, p1.y]) < utm_dist(
            [self.x, self.y], [p2.x, p2.y]):
            return p1x
        return p2

    def move(self, metres, target):
        """Move the robot given a distance and bearing.

        Args:
            metres: The distance the robot to move in metres.
            target: The next node in the route. It's current target
                coordinates.
        """
        target_loc = bearing_distance_from_coordinates(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, {
                EASTING: target[0],
                NORTHING: target[1],
                ELEVATION: 0
            })
        # If the heading is similar to the target
        # and the target is less than the given LiDAR range
        # move directly to the target as there are no obstacles
        # in the way
        if self.heading == target_loc[
                'bg'] and target_loc['dist_2d'] < move_dist:
            self.clear_q()
            metres = (target_loc['dist_2d'] - 0.05)
        pos = coordinates_from_bearing_distance(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, self.heading, metres)
        not_allowed = False
        # Check if movement would result in passing through wall
        for n in range(len(self.nogo_polys)):
            if self.nogo_polys[n].contains(Point([pos['e'], pos['n']])):
                not_allowed = True
            path = LineString([[self.x, self.y], [pos['e'], pos['n']]])
            if path.crosses(self.nogo_polys[n]):
                not_allowed = True
            if path.crosses(self.per_poly):
                not_allowed = True
        if not self.per_poly.contains(Point([pos['e'], pos['n']])):
            not_allowed = True
        # If so move away from the object
        if not_allowed:
            pos = coordinates_from_bearing_distance(
                {
                    EASTING: self.x,
                    NORTHING: self.y,
                    ELEVATION: 0
                }, self.heading, -0.15)
            self.dist_travelled += 0.15
            if REAL_TIME:
                time.sleep(0.15 / 0.89)
        else:
            self.dist_travelled += metres
            if REAL_TIME:
                time.sleep(metres / 0.89)

        self.x = pos['e']
        self.y = pos['n']
        self.tries += 1

    def find_target(self, nogos, target):
        """A helper function to re-orientate the robot to the next
        node in the route.

        Args:
            target: The robot's current target.
        """
        target_loc = bearing_distance_from_coordinates(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, {
                EASTING: target[0],
                NORTHING: target[1],
                ELEVATION: 0
            })
        # Look at target and iteratively turn until not objects in the way
        if DIRECT:
            self.heading = target_loc['bg']
        # Keep current heading and turn towards target until objects in view
        else:
            target_heading = target_loc['bg']
            # Temporary fix to reduce getting stuck on flat walls
            if target_heading > 340 and self.tries >= 3:
                target_heading = 360 - target_heading
            objs = self.detect_objects([nogos], target)
            no_target = 1
            while len(objs) <= 0:
                if target_heading < self.heading:
                    self.heading += (2 * no_target)
                else:
                    self.heading -= (2 * no_target)
                objs = self.detect_objects([nogos], target)
                if abs(self.heading - target_heading) <= 3 and len(objs) <= 0:
                    self.heading = target_heading
                    break
                else:
                    no_target = -1

    def get_detected_line(self, target, nogos, heading, centre_line,
                          test_shape, current, img, path):
        """Get the points detected by the LiDAR.

        Depending on the number of points return either a polygon,
        line, or single point. Whilst there are no detected obstacles,
        move forward - try in the direction of the target first, if
        obstacles then return to original given heading.

        Args:
            target: The current target node in the route.
            nogos: The list of bounds - nogos and perimeter
            heading: The given heading to try.
            test_shape: The bounds of the area.
            current: The current positon in the route - for printing
                the graph
            img: The current image number - for printing the graph
            path: The given coverage route.

        Returns:
            The detected points as a shape, line, or single point.
        """
        self.find_target(nogos, path[target])
        n = nogos.copy()
        n.append(test_shape)
        objs = self.detect_objects([n], path[target])
        if len(objs) != 0:
            self.heading += heading
            objs = self.detect_objects([n], path[target])
        # If enough to make a shape, take the bounds of that shape
        if len(objs) == 0:
            return None, img
        if len(objs) >= 2:
            detected_line = LineString(objs)
            detected_line = LineString(
                [detected_line.coords[0], detected_line.coords[-1]])
        elif len(objs) == 1:
            detected_line = objs[0]
        return detected_line, img


def printable_detected_ends(ends):
    if len(ends) <= 2:
        return []
    lines = []
    len_d = 0
    while len_d < len(ends) - 1:
        line = [list(ends)[len_d]]
        while utm_dist(
            [list(ends)[len_d].x, list(ends)[len_d].y],
            [list(ends)[len_d + 1].x,
             list(ends)[len_d + 1].y]) < 0.5 and len_d < len(list(ends)) - 2:
            line.append(list(ends)[len_d + 1])
            len_d += 1
        if len(line) >= 2:
            lines.append([[
                LineString(line).coords[0][0],
                LineString(line).coords[0][1]
            ], [
                LineString(line).coords[-1][0],
                LineString(line).coords[-1][1]
            ]])
        elif len(line) == 1:
            lines.append([[line[0].x, line[0].y], [
                line[0].x,
                line[0].y,
            ]])

        len_d += 1
    return lines


def print_graph(mower, test_shape, nogos, path, current, target, img,
                detected):
    if not PRINT:
        return 0

    gpd.GeoSeries(mower.lidar_range()[1]).plot()
    gpd.GeoSeries(mower.lidar_range()[1]).plot()

    plt.plot(path[:, 0],
             path[:, 1],
             linestyle='dotted',
             alpha=0.5,
             color='orange')
    for nogo in nogos:
        plt.plot(nogo[:, 0], nogo[:, 1], color='black')
    plt.plot(test_shape[:, 0], test_shape[:, 1], color='blue')
    plt.plot(mower.visited[:, 0], mower.visited[:, 1], color='blue')
    centre_line = mower.lidar_range()[0][int(lidar_range / 2)]

    plt.plot(*centre_line.xy)
    # for p in random.sample(mower.detected_ends,
    #                        min(50, len(mower.detected_ends))):
    print_ends = printable_detected_ends(list(set(mower.detected_points)))
    print("Printing " + str(len(print_ends)) + " detected ends")
    for p in print_ends:
        plt.plot(np.array(p)[:, 0], np.array(p)[:, 1], color='red')

    if mower.is_off_course():
        mower_colour = 'red'
    else:
        mower_colour = 'blue'
    plt.scatter(mower.x, mower.y, color=mower_colour)

    if detected is not None:
        plt.scatter(detected.xy[0], detected.xy[1], color='red')

    plt.scatter(path[current, 0], path[current, 1], color='blue')
    plt.scatter(path[target, 0], path[target, 1], color='orange')
    plt.axis('off')
    plt.savefig("./Imgs/look_position_" + str(img) + ".png")
    plt.close('all')


def shift_float(num):
    # return num
    return (num * (10))


def shift_int(num):
    # return num  # m
    return (num / (10))  # cm


def utm_dist(p1, p2):
    """Short wrapper function to reduce lines of code.
    """
    if (type(p1) is list
            or type(p1) is np.ndarray) and (type(p2) is list
                                            or type(p2) is np.ndarray):
        dist = bearing_distance_from_coordinates(
            {
                EASTING: p1[0],
                NORTHING: p1[1],
                ELEVATION: 0
            }, {
                EASTING: p2[0],
                NORTHING: p2[1],
                ELEVATION: 0
            })
        return dist['dist_2d']
    elif type(p1) is Point and type(p2) is Point:
        dist = bearing_distance_from_coordinates(
            {
                EASTING: p1.x,
                NORTHING: p1.y,
                ELEVATION: 0
            }, {
                EASTING: p2.x,
                NORTHING: p2.y,
                ELEVATION: 0
            })
        return dist['dist_2d']


def collinear(p0, p1, p2):
    """Determine if three points are collinear using the slope method
    """
    x1, y1 = p1[0] - p0[0], p1[1] - p0[1]
    x2, y2 = p2[0] - p0[0], p2[1] - p0[1]
    return abs(x1 * y2 - x2 * y1) < sys.float_info.epsilon


def utm_bearing(p1, p2):
    """Short wrapper function to reduce lines of code.
    """
    dist = bearing_distance_from_coordinates(
        {
            EASTING: p1[0],
            NORTHING: p1[1],
            ELEVATION: 0
        }, {
            EASTING: p2[0],
            NORTHING: p2[1],
            ELEVATION: 0
        })
    return dist['bg']


def to_utm(points):
    """Converts lat/long points to UTM equivalents.

    During this conversion it is necessary for the grids to be the
    same to reduce error. It is unlikely in our application for them
    to be different due to the size of each region but it is checked
    and a warning given nonetheless. 

    """
    xy_shape = points
    zone_nums = []
    zone_lets = []
    for i in range(len(points)):
        try:
            u = np.array(utm.from_latlon(points[i, 0], points[i, 1]))
        except utm.error.OutOfRangeError:
            print("To UTM failed, assuming coordinates are not GPS...")
            return points
        zone_nums.append(int(u[2]))
        zone_lets.append(u[3])
        xy_shape[i, 0] = u[0]
        xy_shape[i, 1] = u[1]

    zone_nums = list(set(zone_nums))
    zone_lets = list(set(zone_lets))
    if len(zone_nums) > 1 or len(zone_lets) > 1:
        input(
            "This region spans for than one region, to continue press any key, but know accuracy may be affected..."
        )
    return xy_shape


def get_closest_intersect(mower, detected_line, lidar):
    """

    Considering all LiDAR lines in this region, which point is closest
    to the robot.

    Args:
        mower: When determining which way to move the robot should
            ideally move away from the closest point as this will
            result in faster and more efficient obstacle avoidance.
            All points must be considered due to irregular shapes, but
            the closest is the primary concern.
        detected_line: The current detected line on a shape defined by
            the two end points.
        lidar: The set of LiDAR lines for a given region (left-side,
            right-side, centre)

    Returns:
        The closest distance from robot to detected point, and whether
        the region intersects at all.
    """
    dists = []
    dist = 0.5
    intersects = False
    for line in lidar:
        if line.intersects(detected_line):
            intersects = True
            inter = line.intersection(detected_line)
            dist = bearing_distance_from_coordinates(
                {
                    EASTING: mower.x,
                    NORTHING: mower.y,
                    ELEVATION: 0
                }, {
                    EASTING: inter.x,
                    NORTHING: inter.y,
                    ELEVATION: 0
                })
            dists.append(dist['dist_2d'])
    if len(dists) > 0:
        dist = min(dists)
    return dist, intersects


def lidar_intersecting(centre, left, right, mower, detected_line):
    """Determine if the LiDAR is detecting an object and if so, where
    on the LiDAR range.

    Depending on where the object is in the LiDAR's range the robot
    will want to move differently i.e. if the object is to the right
    then the robot will want to move left.

    Args:
        centre: The centre line of the LiDAR
        left: The left-most line of the LiDAR
        right: The right-most line of the LiDAR
        mower: The lawn mower.
        detected_line: The line detected by the robot.

    Returns:
        If a line has been detected and in which direction.
    """
    l_dist, l = get_closest_intersect(mower, detected_line, left)
    r_dist, r = get_closest_intersect(mower, detected_line, right)
    c_dist, c = get_closest_intersect(mower, detected_line, centre)

    _, l_most = get_closest_intersect(mower, detected_line, left[0:3])
    _, r_most = get_closest_intersect(mower, detected_line, right[-3:-1])

    # If the mower is too close to an object to move in that direction
    if l_dist <= (move_dist / 4) or r_dist <= (move_dist /
                                               4) or c_dist <= (move_dist / 4):
        print("too close")
        return True, OBJECT_TO_CENTRE
    if c:
        print("centre")
        return True, OBJECT_TO_CENTRE

    # This should be handled when collecting points and merging close
    # by points, but if the considered LiDAR range is equal to the
    # distance considered when merging then this may be quicker
    if l_most and r_most:
        print("left and right - won't fit")
        return True, OBJECT_TO_CENTRE
    if l and r:
        if l_dist < r_dist:
            print("left closest")
            return True, OBJECT_TO_LEFT
        else:
            print("right closest")
            return True, OBJECT_TO_RIGHT

    if r:
        print("right")
        return True, OBJECT_TO_RIGHT
    if l:
        print("left")
        return True, OBJECT_TO_LEFT

    return False, None


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
              img, right_bear, left_bear, centre_bear):
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
            lidar_intersect, side = lidar_intersecting(centre_lines,
                                                       left_lines, right_lines,
                                                       mower, detected_line)
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
                lidar_intersect, side = lidar_intersecting(
                    centre_lines, left_lines, right_lines, mower,
                    detected_line)
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
        print_graph(mower, test_shape, nogos, path, current, target, img,
                    detected_line)
        img += 1
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


def signal_handler(sig, frame):
    global SIGINT
    print("Cancelling run on next main iteration...")
    SIGINT = 1


def main(to_test, run_num):
    global SIGINT
    # NOTE: Please ensure the perimeter is name obstacle_0.out
    # will all other nogo-zones in ascending order e.g. obstacle_1.out, ...

    # Delete all old image files
    files = glob.glob('./Imgs/*.png')
    for f in files:
        os.remove(f)
    warnings.filterwarnings('error')

    # Load objects and perimeter in Lat, Long format
    test_shape = np.loadtxt("./perimeter.out", dtype=float, delimiter=",")
    test_shape = to_utm(test_shape)
    path = np.loadtxt("./route.out", dtype=float, delimiter=",")
    path_len = len(path)
    current = 0  # int(path_len / 2) + 125
    skipped = 0
    target = current + 1

    # Init robot
    mower = Robot(path[current, 0], path[current, 1], target)
    mower.per_poly = Polygon(test_shape)
    mower.visited = np.vstack((mower.visited, [path[0, 0], path[0, 1]]))
    mower.lidar_range()

    # Generate or read nogos
    nogos = []
    if to_test is False:  # Read in given obstacle files
        nogo_files = sorted(list(glob.glob("obstacle*")))
        print(nogo_files)
        for i in nogo_files:
            nogos.append(np.loadtxt(i, dtype=float, delimiter=","))
        print(nogos)
        for i in range(len(nogos)):
            nogos[i] = to_utm(nogos[i])
        for n in range(len(nogos)):
            if len(nogos[n]) > 2:
                mower.nogo_polys.append(Polygon(nogos[n]))
            else:
                mower.nogo_polys.append(LineString(nogos[n]))
    else:  # Generate random obstacles within the given perimeter
        poly = Polygon(test_shape)
        min_x, min_y, max_x, max_y = poly.bounds

        num_polygons = int(random.uniform(1, 4))
        print("Generating " + str(num_polygons) + " random nogos...")
        while len(nogos) < num_polygons:
            width = random.uniform(1, 5)
            height = random.uniform(1, 5)
            rand_x = random.uniform(min_x, max_x)
            rand_y = random.uniform(min_y, max_y)

            left = Point([rand_x, rand_y])
            bottom = Point([rand_x, rand_y - height])
            right = Point([rand_x + width, rand_y - height])
            top = Point([rand_x + width, rand_y])

            new_poly = Polygon([left, bottom, right, top])

            if poly.contains(new_poly):
                print("Generated nogo " + str(len(nogos) + 1))
                nogos.append(np.array(new_poly.exterior.coords))
                np.savetxt('./Tests/' + str(run_num) + '_obstacle_' +
                           str(len(nogos)) + '.out',
                           np.array(new_poly.exterior.coords),
                           delimiter=',')
                mower.nogo_polys.append(new_poly)

    # Initial plot to view and confirm shapes and mower position are correct
    gpd.GeoSeries(mower.lidar_range()[1]).plot()
    plt.plot(test_shape[:, 0], test_shape[:, 1])
    centre_line = mower.lidar_range()[0][int(lidar_range / 2)]
    plt.plot(*centre_line.xy)
    for nogo in nogos:
        plt.plot(nogo[:, 0], nogo[:, 1])
    plt.scatter(path[current, 0], path[current, 1])
    plt.scatter(mower.x, mower.y, color='blue')
    plt.scatter(path[target, 0], path[target, 1])
    plt.show()

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
            if not access or apath == [[mower.x, mower.y]
                                       ] or apath == prev_apath:
                current += 1
                skipped += 1
                target += 1
                print("Path not possible, skipping checkpoint")
            else:
                print("Path may be possible, trying route")
                print_graph(mower, test_shape, nogos, np.array(apath), 0, 1,
                            img, None)
                img += 1
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

    s = gpd.GeoSeries([
        LineString(
            gpd.points_from_xy(x=mower.visited[:, 0], y=mower.visited[:, 1])),
    ])

    f, ax = plt.subplots()
    plt.axis('off')
    s.buffer(0.15).plot(alpha=0.5, ax=ax)

    print("Got to point " + str(current) + " out of " + str(path_len) +
          " had to skip " + str(skipped))
    print("Total area: " + str(Polygon(test_shape).area) + " m^2")
    print("Total area covered: " + str(s.buffer(0.15)[0].area) + " m^2")
    missed = 0
    for n in mower.nogo_polys:
        missed += n.area
    print("Total area inside no-go: " + str(missed) + " m^2")
    print("Total distance travelled: " + str(mower.dist_travelled) + " m")

    f = open("./Tests/" + str(run_num) + ".csv", 'w')
    writer = csv.writer(f)
    writer.writerow(
        [skipped, missed,
         s.buffer(0.15)[0].area, mower.dist_travelled])
    f.close()

    plt.plot(test_shape[:, 0], test_shape[:, 1])
    for nogo in nogos:
        plt.plot(nogo[:, 0], nogo[:, 1])
    # global detected_ends
    # print(len(detected_ends))
    mower.reduce_detected_points()
    for p in list(set(mower.detected_ends)):
        plt.plot(p.array()[:, 0], p.array()[:, 1], color='red')
    if to_test is None:
        plt.savefig("./Imgs/end.png")
    else:
        plt.savefig("./Imgs/" + str(run_num) + "_end.png")


if __name__ == "__main__":
    profiler = cProfile.Profile()
    profiler.enable()
    runs = 1
    if '-t' in sys.argv:
        to_test = True
        runs = sys.argv[sys.argv.index('-t') + 1]
    else:
        to_test = False
    if not '-v' in sys.argv:
        sys.stdout = open(os.devnull, 'w')
    if '-p' in sys.argv:
        PRINT = True
    if '-d' in sys.argv:
        DIRECT = True
    if '-r' in sys.argv:
        REAL_TIME = True
        move_dist = 0.1
    for i in range(int(runs)):
        main(to_test, i)
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats('tottime')
    stats.strip_dirs()
    stats.print_stats()
