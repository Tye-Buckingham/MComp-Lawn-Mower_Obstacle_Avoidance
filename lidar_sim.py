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

import glob
import math
import os
import sys
import warnings

import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
import utm
from mathutils import Vector
from mathutils.geometry import intersect_point_line
from shapely.geometry import LineString, Point, Polygon

from surveytoolbox.bdc import bearing_distance_from_coordinates
from surveytoolbox.cbd import coordinates_from_bearing_distance
from surveytoolbox.config import BEARING, EASTING, ELEVATION, NORTHING
from surveytoolbox.fmt_dms import format_as_dms

lidar_range = 60
lidar_dist = 1
move_dist = 0.5
lidar_width = 15

detected_ends = []

QUEUE_LEN = 10
ON_COURSE = 0
OFF_COURSE = 1
Q_SIZE = 10
MAX_SPEED = 0.6

OFF_COURSE_TARGET = -1
OBJECT_TO_LEFT = -1
OBJECT_TO_RIGHT = 1
OBJECT_TO_CENTRE = 0


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
        if self.per_on_course() < 0.6:
            return OFF_COURSE
        return ON_COURSE

    def clear_q(self):
        """Resets the queue. Used when the robot confirms reaching a
        checkpoint - may not be used in real application.
        """
        self.inside = [0] * Q_SIZE

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
                        [points[i].x, points[i].y]) < 0.5:
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
        global detected_ends
        lidar_lines = self.lidar_range()[0]
        for nogo in nogos[0]:  # For each object
            for i in range(-1, len(nogo) - 1):  # Loop over each corner
                # Create a solid edge to the object
                edge = LineString([(nogo[i][0], nogo[i][1]),
                                   (nogo[i + 1][0], nogo[i + 1][1])])
                for line in lidar_lines:
                    if line.intersects(edge):
                        detected_ends.append(line.intersection(edge))
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
            points = self.find_nearest(points)

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
        if self.heading == target_loc['bg'] and target_loc['dist_2d'] < 0.5:
            self.clear_q()
            metres = (target_loc['dist_2d'] - 0.05)
        pos = coordinates_from_bearing_distance(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, self.heading, metres)
        inside = False
        # Check if movement would result in passing through wall
        for n in range(len(self.nogo_polys)):
            if self.nogo_polys[n].contains(Point([pos['e'], pos['n']])):
                inside = True
        # If so move away from the object
        if inside:
            pos = coordinates_from_bearing_distance(
                {
                    EASTING: self.x,
                    NORTHING: self.y,
                    ELEVATION: 0
                }, self.heading, -0.15)

        self.x = pos['e']
        self.y = pos['n']

    def find_target(self, target):
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
        self.heading = target_loc['bg']

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
        self.find_target(path[target])
        objs = self.detect_objects([nogos], path[target])
        if len(objs) != 0:
            self.heading += heading
            objs = self.detect_objects([nogos], path[target])
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


def print_graph(mower, test_shape, nogos, path, current, target, img,
                detected):

    gpd.GeoSeries(mower.lidar_range()[1]).plot()

    plt.plot(path[:, 0],
             path[:, 1],
             linestyle='dotted',
             alpha=0.5,
             color='orange')
    plt.plot(test_shape[:, 0], test_shape[:, 1])
    plt.plot(mower.visited[:, 0], mower.visited[:, 1], color='blue')
    centre_line = mower.lidar_range()[0][int(lidar_range / 2)]
    plt.plot(*centre_line.xy)
    if mower.is_off_course():
        mower_colour = 'red'
    else:
        mower_colour = 'blue'
    plt.scatter(mower.x, mower.y, color=mower_colour)

    if detected is not None:
        plt.scatter(detected.xy[0], detected.xy[1], color='red')
    for nogo in nogos:
        plt.plot(nogo[:, 0], nogo[:, 1])

    plt.scatter(path[current, 0], path[current, 1])
    plt.scatter(path[target, 0], path[target, 1])
    plt.axis('off')
    plt.savefig("./Imgs/look_position_" + str(img) + ".png")
    plt.close('all')


def utm_dist(p1, p2):
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
        u = np.array(utm.from_latlon(points[i, 0], points[i, 1]))
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

    if l_dist <= 0.31 or r_dist <= 0.31 or c_dist <= 0.31:
        return True, OBJECT_TO_CENTRE
    if c:
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

        # If no detected objects, break to move forward
        if detected_line is None:
            mower.move(move_dist, path[target])
            # The 'path' is different when off course due to the new determined
            # back-on-track point
            if target == OFF_COURSE_TARGET:
                t = 1
            else:
                t = target
            mower.enqueue(mower.is_outside_buffer(path, current, t))
            mower.visited = np.vstack((mower.visited, [mower.x, mower.y]))
            gpd.GeoSeries(mower.lidar_range()[1]).plot()
            print_graph(mower, test_shape, nogos, path, current, target, img,
                        detected_line)
            img += 1
            m = 0
            # Returning to re-calculate temporary target
            if mower.is_off_course():
                return img
            continue
        centre_lines = mower.lidar_range()[0][int(lidar_range *
                                                  (1 / 3)):int(lidar_range *
                                                               (2 / 3))]
        left_lines = mower.lidar_range()[0][0:int(lidar_range * (1 / 3))]
        right_lines = mower.lidar_range()[0][int(lidar_range * (2 / 3)):-1]
        lidar_intersect, side = lidar_intersecting(centre_lines, left_lines,
                                                   right_lines, mower,
                                                   detected_line)
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
                target, nogos, bear[m], line, test_shape, current, img, path)

            # Once no obstacles are found, break to move foward
            if detected_line is None:
                break
            centre_lines = mower.lidar_range()[0][int(lidar_range *
                                                      (1 /
                                                       3)):int(lidar_range *
                                                               (2 / 3))]
            left_lines = mower.lidar_range()[0][0:int(lidar_range * (1 / 3))]
            right_lines = mower.lidar_range()[0][int(lidar_range * (2 / 3)):-1]
            lidar_intersect, side = lidar_intersecting(centre_lines,
                                                       left_lines, right_lines,
                                                       mower, detected_line)
            img += 1
            print(bear[m])
            m += 1
            # If no turning options left, end. - Handle points impossible to reach
            if m == len(bear) - 1:
                print(side)
                raise Exception("Couldn't get to point")
        mower.move(move_dist, path[target])
        if target == OFF_COURSE_TARGET:
            t = 1
        else:
            t = target
        mower.enqueue(mower.is_outside_buffer(path, current, t))
        mower.visited = np.vstack((mower.visited, [mower.x, mower.y]))
        gpd.GeoSeries(mower.lidar_range()[1]).plot()
        print_graph(mower, test_shape, nogos, path, current, target, img,
                    detected_line)
        img += 1
        m = 0
        # If the mower is off course return to pick new point on line
        if mower.is_off_course():
            return img
        # If mower is no longer off course
        if not mower.is_off_course() and target == -1:
            return img

    return img


def main():
    files = glob.glob('./Imgs/*.png')
    for f in files:
        os.remove(f)
    warnings.filterwarnings('error')
    # Load objects and perimeter in Lat, Long format
    nogos = []
    nogo_files = list(glob.glob("obstacle*"))
    print(nogo_files)
    for i in nogo_files:
        nogos.append(np.loadtxt(i, dtype=float, delimiter=","))
    print(nogos)
    test_shape = np.loadtxt("./perimeter.out", dtype=float, delimiter=",")
    path = np.loadtxt("./route.out", dtype=float, delimiter=",")
    path_len = len(path)
    current = 0  # int(path_len / 2)
    target = current + 1

    # Convert Lat, Long to Northing, Easting (UTM)
    test_shape = to_utm(test_shape)
    for i in range(len(nogos)):
        nogos[i] = to_utm(nogos[i])

    mower = Robot(path[current, 0], path[current, 1], target)
    for n in range(len(nogos) - 1):
        mower.nogo_polys.append(Polygon(nogos[n]))
    mower.visited = np.vstack((mower.visited, [path[0, 0], path[0, 1]]))

    mower.lidar_range()

    ## Initial plot to view and confirm shapes and mower position are correct
    gpd.GeoSeries(mower.lidar_range()[1]).plot()
    plt.plot(test_shape[:, 0], test_shape[:, 1])
    # plt.scatter(*detected_line.xy, color='red')
    centre_line = mower.lidar_range()[0][int(lidar_range / 2)]
    plt.plot(*centre_line.xy)
    for nogo in nogos:
        plt.plot(nogo[:, 0], nogo[:, 1])
    plt.scatter(path[current, 0], path[current, 1])
    plt.scatter(mower.x, mower.y, color='blue')
    plt.scatter(path[target, 0], path[target, 1])
    plt.show()

    # If the robot detects an object to the right, turn left
    right_bear = list(range(1, 180, 2))
    # If the robot detects an object to the left, turn right
    left_bear = list(range(-1, -180, -2))
    # If the robot detects an object in front, try left and right
    centre_bear = [
        val for pair in zip(list(range(0, 180, 4)), list(range(0, -180, -4)))
        for val in pair
    ]
    img = 0

    while target < path_len:
        # If reached target within region of inaccuracy
        if utm_dist([mower.x, mower.y], path[target]) <= 0.2:
            current += 1
            target += 1
            continue
        if mower.is_off_course():
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
            img = avoidance(mower, np.array([path[current], path[target], p]),
                            OFF_COURSE_TARGET, nogos, centre_line, test_shape,
                            0, img, right_bear, left_bear, centre_bear)
        else:
            img = avoidance(mower, path, target, nogos, centre_line,
                            test_shape, current, img, right_bear, left_bear,
                            centre_bear)

    s = gpd.GeoSeries([
        LineString(
            gpd.points_from_xy(x=mower.visited[:, 0], y=mower.visited[:, 1])),
    ])

    f, ax = plt.subplots()
    plt.axis('off')
    s.buffer(0.15).plot(alpha=0.5, ax=ax)
    plt.plot(test_shape[:, 0], test_shape[:, 1])
    # for nogo in nogos:
    #     plt.plot(nogo[:, 0], nogo[:, 1])
    # global detected_ends
    # print(len(detected_ends))
    # detected_ends = set(detected_ends)
    # for p in detected_ends:
    #     plt.scatter(p.x, p.y)
    plt.savefig("./Imgs/end.png")


if __name__ == "__main__":
    main()
