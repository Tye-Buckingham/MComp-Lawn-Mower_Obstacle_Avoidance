""" Robot is the main class for controlling the robotic mower and determining
detected objects with the LiDAR - the LiDAR may be seperated out in future
development.

Note: The LiDAR values do not have to be the maximum values of the sensor
used, a subset can be considered i.e. one can consider only the front section
when using a 360 LiDAR.
"""
import sys
import time

import numpy as np
import rclpy
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from rclpy.node import Node
from shapely.geometry import LineString, Point, Polygon
from std_msgs.msg import String

from .consts import *
from .ends import Ends
from .lds import LFCDLaser
from .utm_func import utm_bearing, utm_bearing_dist, utm_coords, utm_dist

DETECTED_POINTS_MULTIPLIER = 1


class MovePub(Node):

    def __init__(self):
        self.cmd_index = 0
        super().__init__('mower_pub')
        self.publisher_ = self.create_publisher(RobotSendCMD,
                                                'Robot_Bridge_CMD', 10)
        self.publisher_

    def publish_move(self, heading, distance):
        cmd_msg = RobotSendCMD()
        cmd_msg.cmd_char = ord('d')
        cmd_msg.arg1 = 1500.0  # RPM should be based on time to move 10cm to allow enough time to process
        cmd_msg.arg2 = 0.1  # 10 cm
        self.publisher_.publish(cmd_msg)
        self.get_logger().info("Sent CMD message: %s" % cmd_msg)


class RTKSub(Node):

    def __init__(self, mower):
        super().__init__('rtk_sub')
        self.subscription = self.create_subscription(String, 'topic',
                                                     self.listener_callback, 1)
        self.subscription
        self.assigned = mower

    def listener_callback(self, msg):
        self.get_logger().info('I heard: %s' % msg.data)


class IMUSub(Node):

    def __init__(self, mower):
        super().__init__('imu_sub')
        self.subscription = self.create_subscription(String, 'topic',
                                                     self.listener_callback, 1)
        self.subscription
        self.assigned = mower

    def listener_callback(self, msg):
        self.get_logger().info('I heard: %s' % msg.data)


class Robot():
    """The Robot class is the simulated version of a lawn mower
    used for testing obstacle detection, mapping, and
    area coverage.
    """

    def __init__(self, x, y, target, Q_SIZE, lidar_range, lidar_width,
                 lidar_dist, move_dist, try_count, obj_gap, REAL_TIME):
        # Considerable number of members for testing -
        # may not need all in future versions
        rclpy.init(args=None)
        self.move_pub = MovePub()
        self.imu_sub = IMUSub(self)
        self.rtk_sub = RTKSub(self)
        self.x = x  # TODO ROS subscriber to RTK to get [x, y]
        self.y = y
        self.visited = np.empty((0, 2))
        self.heading = float()
        self.target = target
        self.inside = [0] * Q_SIZE
        self.Q_SIZE = Q_SIZE
        self.nogo_polys = []
        self.per_poly = None
        self.dist_travelled = 0
        self.detected_ends = []
        self.detected_points = []
        self.tries = 0
        self.lidar_range = lidar_range
        self.lidar_dist = lidar_dist
        self.lidar_width = lidar_width
        self.move_dist = move_dist
        self.try_count = try_count
        self.obj_gap = obj_gap
        self.REAL_TIME = REAL_TIME
        self.lidar = LFCDLaser("/dev/ttyUSB0", 230400)  # lds lidar driver

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
        if self.per_on_course() < 0.90:
            return OFF_COURSE
        return ON_COURSE

    def clear_q(self):
        """Resets the queue. Used when the robot confirms reaching a
        checkpoint - may not be used in real application.
        """
        self.inside = [0] * self.Q_SIZE

    def seperate_lidar_ranges(self):
        """Movement depends on where the obstacle is detected by the
        LiDAR i.e. if detected on left the robot will favour right
        movement.

        The LiDAR is split into ranges to give a balance of direction
        favouring. Favouring a side too much may result in quicker
        movement decisions, but often results in the mower getting
        stuck more often.
        """
        centre_lines = self.lidar_lines()[0][int(self.lidar_range *
                                                 (1 /
                                                  3)):int(self.lidar_range *
                                                          (2 / 3))]
        left_lines = self.lidar_lines()[0][0:int(self.lidar_range * (1 / 3))]
        right_lines = self.lidar_lines()[0][int(self.lidar_range * (2 / 3)):-1]
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
        if len(self.visited) > self.try_count and self.tries > self.try_count:
            dist = utm_dist([self.x, self.y], self.visited[-(self.try_count)])
            if dist <= self.move_dist:
                return 0
        # tries = int((dist / move_dist) * 1.1)
        # print("Try " + str(self.tries) + " out of " + str(tries))
        # if dist <= 1:
        #     tries = 0
        # else:
        #     tries = self.tries + 1
        return self.tries + 1

    def order_line(self, line):
        """Takes a LineString and orders the given points from the
        starting boundary.

        Ordering the line is needed to reduce the number of points to
        the resolution needed for the A* grid. Only points which are
        considered one object (less than a given distance away) are
        kept as one line, points which are too far away are added to
        'skipped', the process is repeated for these skipped points
        until are all in a given line.

        Args:
            line: The LineString needing to be sorted and segmented.

        Returns:
            If no points are found None is returned, this needs to be
            checked as not all points are necessarily part of a line.
        """
        ordered_line = [Point(line.boundary.geoms[0])]
        skipped = []
        coords_len = len(line.coords)
        next_point = None
        while (len(ordered_line) + len(skipped)) < coords_len:
            dist = sys.float_info.max
            for i in line.coords:
                # If already added, ignore
                if Point(i) == ordered_line[len(ordered_line) -
                                            1] or Point(i) in ordered_line:
                    continue
                # If already skipped, ignore
                if Point(i) in skipped:
                    continue
                # If too far away to be one object, add to skipped
                if ordered_line[len(ordered_line) - 1].distance(
                        Point(i)) > self.obj_gap:
                    skipped.append(Point(i))
                    continue
                # If close enough, consider
                if ordered_line[len(ordered_line) - 1].distance(
                        Point(i)) < dist:
                    dist = ordered_line[len(ordered_line) - 1].distance(
                        Point(i))
                    next_point = Point(i)
            # If no points were found to be close enough
            if next_point is None:
                break
            # Add the closest point to the previously added point
            ordered_line.append(next_point)
        if len(ordered_line) < 2:
            return None, skipped
        return LineString(ordered_line), skipped

    def detected_points_to_lines(self):
        """Reduce the detected points to simpler linestrings.

        Using all detected points quickly gets out of hand, so lines
        are used instead. The end points of these lines can be used to
        replace the current detected points to save memory and
        processing time.
        """
        lines = []
        self.detected_points = list(set(self.detected_points))
        # List is kept to keep track of added points
        added = []
        for i in self.detected_points:
            if i in added:
                continue
            line = Point(i)
            while True:
                # If a new point is added to line then reset loop
                reset = False
                for j in self.detected_points:
                    if j in added or i == j:
                        continue
                    if line.distance(j) < self.obj_gap:
                        x, y = line.coords.xy
                        c = list(map(Point, zip(x, y)))
                        c.append(j)
                        line = LineString(c)
                        added.append(j)
                        reset = True
                if not reset:
                    break
            if type(line) == LineString and len(line.coords) >= 2:
                # Get the ordered line and the remaining points
                # not part of said line
                line, remainder = self.order_line(line)
                # 0.1 is used as this is the resolution of the A* grid
                dists = np.arange(0, line.length, 0.1)
                points = [line.boundary.geoms[0]
                          ] + [line.interpolate(d)
                               for d in dists] + [line.boundary.geoms[1]]
                lines.append(LineString(points))
                # Repeat for remainder
                # TODO find a more succinct way to do this without initial step
                # followed by a loop
                while len(remainder) > 1:
                    line, remainder = self.order_line(LineString(remainder))
                    if line is None:
                        break
                    dists = np.arange(0, line.length, 0.1)
                    points = [line.boundary.geoms[0]
                              ] + [line.interpolate(d)
                                   for d in dists] + [line.boundary.geoms[1]]
                    lines.append(LineString(points))

        return lines

    def is_accessible(self, target):
        """When the robot is unable to make progress it may be stuck,
        using it's knowledge of the scene an A* path is generated. If
        no path can be generated then the point is unreachable and
        shall be skipped.
        """

        b = self.per_poly.bounds  # minx, miny, maxx, maxy

        # Build matrix with min-max values
        m_x = int(shift_float(b[2]) - shift_float(b[0])) + 1
        m_y = int(shift_float(b[3]) - shift_float(b[1])) + 1
        matrix = [[1] * max(m_x, m_y)] * max(m_y, m_x)
        matrix = np.array(matrix)

        if False:
            d = self.detected_points_to_lines()
            # Add all points on the reduce lines
            for line in d:
                for point in line.coords:
                    x = int(shift_float(point[0]) - shift_float(b[0]))
                    y = int(shift_float(point[1]) - shift_float(b[1]))
                    matrix[y][x] = -1
        else:
            for point in self.detected_points:
                x = int(shift_float(point.x) - shift_float(b[0]))
                y = int(shift_float(point.y) - shift_float(b[1]))
                matrix[y][x] = -1

        # Add perimeter with interpolate lines to connect
        # TODO could we do this once at the start instead of each time?
        # - memory vs time
        virtual_bounds = []
        per_coords = self.per_poly.exterior.coords
        virtual_bounds.append(per_coords)
        for n in self.nogo_polys:
            virtual_bounds.append(n.exterior.coords)
        for p in range(-1, len(per_coords) - 1):
            line = LineString([per_coords[p], per_coords[p + 1]])
            dist = Point(line.coords[0]).distance(Point(line.coords[1]))
            for i in range(int(dist + 1) * 10):
                point = line.interpolate(i / 10)
                x = int(shift_float(point.x) - shift_float(b[0]))
                y = int(shift_float(point.y) - shift_float(b[1]))
                for i in [-1, 0, 1]:
                    for j in [-1, 0, 1]:
                        try:
                            matrix[y + i][x + j] = -1
                        except IndexError:
                            pass

        # self.print_finder_graph(target, b, matrix)
        grid = Grid(matrix=matrix)
        start = grid.node(int(shift_float(self.x) - shift_float(b[0])),
                          int(shift_float(self.y) - shift_float(b[1])))
        end = grid.node(int(shift_float(target[0]) - shift_float(b[0])),
                        int(shift_float(target[1]) - shift_float(b[1])))

        # AStarFinder, BestFirst, BiAStarFinder, DijkstraFinder,
        # IDAStarFinder, BreadthFirstFinder, MinimumSpanningTree
        finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
        path, runs = finder.find_path(start, end, grid)
        apath = [[self.x, self.y]]
        for p in path:
            if isinstance(p, tuple):
                matrix[p[1]][p[0]] = 2
                # Points need shifting back to original UTM scale
                apath.append([
                    shift_int(p[0] + shift_float(b[0])),
                    shift_int(p[1] + shift_float(b[1]))
                ])
            else:
                matrix[p.y][p.x] = 2
                # Points need shifting back to original UTM scale
                apath.append([
                    shift_int(p.x + shift_float(b[0])),
                    shift_int(p.y + shift_float(b[1]))
                ])

        # self.print_finder_graph(target, b, matrix)
        # apath = self.clean_apath(path, b)
        for i in range(2):
            if len(apath) > 2:
                del apath[2 - 1::2]
        return len(apath) > 1, apath

    def lidar_lines(self):
        """Generate an array of lines at different angles to simulate
        the robot's LiDAR

        Returns:
            inter in index 0 is the LiDAR array used for detection.
            The polygon in index 1 is simply used for the visual
            output.
        """
        # Get the left and right most lines for the visual output
        left = utm_coords([self.x, self.y], (self.heading - self.lidar_width),
                          self.lidar_dist)
        right = utm_coords([self.x, self.y], (self.heading + self.lidar_width),
                           self.lidar_dist)

        inter = []
        # Generate each line with given bearing and length
        for i in range(self.lidar_range):
            pos = utm_coords([self.x, self.y],
                             (self.heading - self.lidar_width) +
                             (i / (self.lidar_range / (self.lidar_width * 2))),
                             self.lidar_dist)
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
                        [points[i].x, points[i].y]) < self.obj_gap:
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

    def detect_virtual_boundaries(self, virtual_bounds, target):
        """Detect if virtual boundary is within range of robot
        
        This method does require the objects to be 'known' but will
        only return those visible to the robot.
        
        Args:
            virtual_bounds: The list of virtual_bounds in the scene, and the perimeter
            target: The robot's current coordinate target, typically
                the next in the given route.
        
        Returns:
            All objects within range of the robot.
        """
        points = []
        lidar_lines = self.lidar_lines()[0]
        for virtual_bound in virtual_bounds[0]:  # For each object
            for i in range(-1,
                           len(virtual_bound) - 1):  # Loop over each corner
                # Create a solid edge to the object
                edge = LineString([(virtual_bound[i][0], virtual_bound[i][1]),
                                   (virtual_bound[i + 1][0],
                                    virtual_bound[i + 1][1])])
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
                        ]) > self.lidar_dist:
                            continue
                        points.append(line.intersection(edge))

        # Once all relevant points are found, determine which to handle first
        # Points are not added to detected_points like in detect_objects
        # as we already know these shapes
        if len(points) > 0:
            points = self.remove_hidden(points)
            points = self.find_nearest(points)

        return points

    def detect_objects(self, virtual_bounds, target):
        """Find all objects within range of the robot's LiDAR.
        
        This method does require the objects to be 'known' but will
        only return those visible to the robot.
        
        Args:
            virtual_bounds: The list of virtual_bounds in the scene.
            target: The robot's current coordinate target, typically
                the next in the given route.
        
        Returns:
            All objects within range of the robot.
        """
        # The interface with the LiDAR system has been stitched together.
        # This is not the most efficient way, re-writing lidar_intersecting
        # to use lds[0] (it's relative angle) instead of points and
        # intersections would be a better idea

        lds_points = self.lidar.poll()  # Get points from LiDAR over serial
        points = []
        for lds in lds_points:
            if lds[1] > self.lidar_dist:  # if too far to consider
                continue
            if lds[1] > utm_dist([self.x, self.y], target):  # if behind target
                continue
            if lds[1] == 0:  # if no object detected
                continue
            if lds[0] < 15 or lds[0] > 345:  # if in-front
                coord = utm_coords([self.x, self.y], self.heading + lds[0],
                                   lds[1])
                points.append(Point([coord['e'], coord['n']]))
        # Once all relevant points are found, determine which to handle first
        if len(points) > 0:
            for p in points:
                self.detected_points.append(p)
            points = self.find_nearest(points)
            self.detected_points = list(set(self.detected_points))
            print("Detected Points: " + str(len(self.detected_points)))
            global DETECTED_POINTS_MULTIPLIER
            if False:
                if len(self.detected_points) > (500 *
                                                DETECTED_POINTS_MULTIPLIER):
                    d = self.detected_points_to_lines()
                    self.detected_points = []
                    for line in d:
                        for p in line.coords:
                            self.detected_points.append(Point(p))
                    if len(self.detected_points) > (
                            500 * DETECTED_POINTS_MULTIPLIER) * 0.5:
                        DETECTED_POINTS_MULTIPLIER += 1
        # Not the best way to do this, should be seperate, but it will do for now
        points.append(self.detect_virtual_boundaries(
            virtual_bounds, target))  # Add virtual obstacles as well
        return points

    def closer_point(self, p1, p2):
        """Finds the closer of two points on a UTM grid
        
        Returns:
            The closest point.
        """
        if utm_dist([self.x, self.y], [p1.x, p1.y]) < utm_dist(
            [self.x, self.y], [p2.x, p2.y]):
            return p1
        return p2

    def move(self, metres, target):
        """Move the robot given a distance and bearing.

        Args:
            metres: The distance the robot to move in metres.
            target: The next node in the route. It's current target
                coordinates.
        """
        target_loc = utm_bearing_dist([self.x, self.y], target)
        # If the heading is similar to the target
        # and the target is less than the given LiDAR range
        # move directly to the target as there are no obstacles
        # in the way
        if self.heading == target_loc[
                'bg'] and target_loc['dist_2d'] < self.move_dist:
            self.clear_q()
            metres = (target_loc['dist_2d'] - 0.05)
        # Potential position
        pos = utm_coords([self.x, self.y], self.heading, metres)
        not_allowed = False
        path = LineString([[self.x, self.y], [pos['e'], pos['n']]])
        # Check if movement would result in passing through nogo boundary
        for n in range(len(self.nogo_polys)):
            # If inside nogo
            if self.nogo_polys[n].contains(Point([pos['e'], pos['n']])):
                not_allowed = True
            # If crossing nogo
            if path.crosses(self.nogo_polys[n]):
                not_allowed = True
            if path.crosses(self.per_poly):
                not_allowed = True
        # If outside perimeter
        if not self.per_poly.contains(Point([pos['e'], pos['n']])):
            not_allowed = True
        # If path crosses over perimeter boundary
        if path.crosses(self.per_poly):
            not_allowed = True
        # If movement not allowed, move away from the object
        if not_allowed:
            pos = utm_coords([self.x, self.y], self.heading, -0.15)
            self.dist_travelled += 0.15
        # Send move command and update position using RTK ROS node
        self.x = pos['e']
        self.y = pos['n']
        self.tries += 1

    def find_target(self, virtual_bounds, target):
        """A helper function to re-orientate the robot to the next
        node in the route.

        Args:
            target: The robot's current target.
        """
        target_loc = utm_bearing([self.x, self.y], target)
        # Keep current heading and turn towards target until objects in view
        target_heading = target_loc
        # Temporary fix to reduce getting stuck on flat walls
        if target_heading > 340 and self.tries >= 3:
            target_heading = 360 - target_heading
        objs = self.detect_objects([virtual_bounds], target)
        no_target = 1
        while len(objs) <= 0:
            if target_heading < self.heading:
                self.heading += (2 * no_target)
            else:
                self.heading -= (2 * no_target)
            objs = self.detect_objects([virtual_bounds], target)
            if abs(self.heading - target_heading) <= 3 and len(objs) <= 0:
                self.heading = target_heading
                break
            else:
                no_target = -1

    def get_detected_line(self, target, virtual_bounds, heading, centre_line,
                          test_shape, current, img, path):
        """Get the points detected by the LiDAR.

        Depending on the number of points return either a polygon,
        line, or single point. Whilst there are no detected obstacles,
        move forward - try in the direction of the target first, if
        obstacles then return to original given heading.

        Args:
            target: The current target node in the route.
            virtual_bounds: The list of bounds - virtual_bounds and perimeter
            heading: The given heading to try.
            test_shape: The bounds of the area.
            current: The current positon in the route - for printing
                the graph
            img: The current image number - for printing the graph
            path: The given coverage route.

        Returns:
            The detected points as a shape, line, or single point.
        """
        self.find_target(virtual_bounds, path[target])
        n = virtual_bounds.copy()
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

    def get_closest_intersect(self, detected_line, lidar):
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
                dist = utm_dist([self.x, self.y], [inter.x, inter.y])
                dists.append(dist)
        if len(dists) > 0:
            dist = min(dists)
        return dist, intersects

    def lidar_intersecting(self, centre, left, right, detected_line):
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
        l_dist, l = self.get_closest_intersect(detected_line, left)
        r_dist, r = self.get_closest_intersect(detected_line, right)
        c_dist, c = self.get_closest_intersect(detected_line, centre)

        _, l_most = self.get_closest_intersect(detected_line, left[0:3])
        _, r_most = self.get_closest_intersect(detected_line, right[-3:-1])

        # If the mower is too close to an object to move in that direction
        if l_dist <= (self.move_dist / 4) or r_dist <= (
                self.move_dist / 4) or c_dist <= (self.move_dist / 4):
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


def shift_float(num):
    """Shifts a UTM value from standard accuracy to desired accuracy.

    Shifting by 10 produces cm level scale. More than this and the
    robot is often engulfed by nearby points, too little and A* thinks
    the robot can fit through small gaps (A* doesn't account for the
    robot's width)

    Args:
        num: The UTM number
    """
    # return num
    return (num * (10))


def shift_int(num):
    """Reverse the shift done by 'shift_float'

    Args:
        num: The shifted UTM value
    """
    # return num  # m
    return (num / (10))  # cm
