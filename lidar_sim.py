import math
import sys

import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
import utm
from shapely.geometry import LineString, Point, Polygon
from surveytoolbox.bdc import bearing_distance_from_coordinates
from surveytoolbox.cbd import coordinates_from_bearing_distance
from surveytoolbox.config import BEARING, EASTING, ELEVATION, NORTHING
from surveytoolbox.fmt_dms import format_as_dms

############### TODO ##################
# Continue moving until reaches end point?
# Need to determine if point is inaccesbile

lidar_range = 60
lidar_dist = 1
move_dist = 0.3


class Robot:

    def __init__(self, x, y, target):
        self.x = x
        self.y = y
        self.visited = np.empty((0, 2))
        self.heading = float()
        self.target = target

    def lidar_range(self):
        left = coordinates_from_bearing_distance(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, (360 - 15) + self.heading, lidar_dist)
        right = coordinates_from_bearing_distance(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, 15 + self.heading, lidar_dist)
        inter = []
        # interval = abs(((360 - 15) + self.heading) - (15 + self.heading)) / 120
        for i in range(lidar_range):
            pos = coordinates_from_bearing_distance(
                {
                    EASTING: self.x,
                    NORTHING: self.y,
                    ELEVATION: 0
                }, ((360 - 15) + self.heading) + (i / 2), lidar_dist)

            inter.append(LineString([(self.x, self.y), (pos['e'], pos['n'])]))

        return [
            inter,
            Polygon([(self.x, self.y), (left['e'], left['n']),
                     (right['e'], right['n']), (self.x, self.y)])
        ]

    def find_nearest(self, points):
        nearest = [points[0]]
        for i in range(1, len(points)):
            # If robot cannot fit in the gap consider it a solid object
            if math.dist([points[i - 1].x, points[i - 1].y],
                         [points[i].x, points[i].y]) < 0.5:
                nearest.append(points[i])
            else:
                # If the new point is closer make new list - tackle closest objects first
                if math.dist(
                    [self.x, self.y], [points[i].x, points[i].y]) < math.dist(
                        [self.x, self.y], [points[i - 1].x, points[i - 1].y]):
                    nearest = [points[i]]
        return nearest

    def detect_objects(self, nogos):
        points = []
        lidar_lines = self.lidar_range()[0]
        for nogo in nogos[0]:
            for i in range(-1, len(nogo) - 1):
                edge = LineString([(nogo[i][0], nogo[i][1]),
                                   (nogo[i + 1][0], nogo[i + 1][1])])
                for line in lidar_lines:
                    if line.intersects(edge):
                        for j in range(len(points)):
                            if get_line([
                                    self.x, self.y
                            ], points[j]) == get_line([
                                    self.x, self.y
                            ], line.intersection(edge)):  # if on the same line
                                # if new one is closer
                                points[i] = self.closer_point(
                                    points[j], line.intersection(edge))
                                continue
                        points.append(line.intersection(edge))
        if len(points) > 0:
            points = self.find_nearest(points)
        return points

    def closer_point(self, p1, p2):
        if math.dist([self.x, self.y], [p1.x, p1.y]) < math.dist(
            [self.x, self.y], [p2.x, p2.y]):
            return p1
        return p2

    def move(self, metres):
        pos = coordinates_from_bearing_distance(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, self.heading, metres)
        self.x = pos['e']
        self.y = pos['n']

    def find_target(self, end):
        target_loc = bearing_distance_from_coordinates(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, {
                EASTING: end[0, 0],
                NORTHING: end[0, 1],
                ELEVATION: 0
            })
        self.heading = target_loc['bg']


def get_detected_line(mower, end, nogos, heading, centre_line, test_shape,
                      start, img):
    mower.find_target(end)
    objs = mower.detect_objects([nogos])
    if len(objs) != 0:
        mower.heading += heading
        objs = mower.detect_objects([nogos])
    # If no object in between target and robot, go straight to it
    while len(objs) == 0:
        mower.move(move_dist)
        mower.visited = np.vstack((mower.visited, [mower.x, mower.y]))
        gpd.GeoSeries(mower.lidar_range()[1]).plot()
        plt.plot(test_shape[:, 0], test_shape[:, 1])
        plt.plot(mower.visited[:, 0], mower.visited[:, 1], color='blue')
        centre_line = mower.lidar_range()[0][int(lidar_range / 2)]
        plt.plot(*centre_line.xy)
        plt.scatter(mower.x, mower.y, color='blue')
        for nogo in nogos:
            plt.plot(nogo[:, 0], nogo[:, 1])
        plt.scatter(start[0, 0], start[0, 1])
        plt.scatter(end[0, 0], end[0, 1])
        plt.savefig("look_position_" + str(img) + ".png")
        plt.clf()
        img += 1
        if abs(mower.x - end[0][0]) < 0.1 and abs(mower.y - end[0][1]) < 0.1:
            return None, img
        objs = mower.detect_objects([nogos])
        # If enough to make a shape, take the bounds
    if len(objs) >= 4:
        detected_points = Polygon(objs)
        detected_ends = detected_points.bounds
        detected_line = LineString([[detected_ends[0], detected_ends[1]],
                                    [detected_ends[2], detected_ends[3]]])
    elif len(objs) == 1:
        detected_line = objs[0]
        # Otherwise take a line
    else:
        detected_line = LineString(objs)
    return detected_line, img


def to_utm(points):
    xy_shape = points
    zone_nums = []
    zone_lets = []
    for i in range(len(points)):
        u = np.array(utm.from_latlon(points[i, 0], points[i, 1]))
        zone_nums.append(int(u[2]))
        zone_lets.append(u[3])
        xy_shape[i, 0] = u[0]
        xy_shape[i, 1] = u[1]

    return xy_shape


def lidar_intersecting(centre, left, right, mower, detected_line):
    if centre.intersects(detected_line):
        return True
    if left.intersects(detected_line):
        return True
    if right.intersects(detected_line):
        return True
    return False


def get_line(a, b):
    if (b.x - a[0]) == 0:
        a[0] += np.finfo(np.float64).min
    m = (b.y - a[1]) / (b.x - a[0])
    c = a[1] - (m * a[0])
    return m, c


# def cartesian_bearing(p1, p2):
#     dy =
#     return math.degrees(math.atan2(dy,dx))+360)%360


def main():
    test_shape = np.array([[]])

    nogos = list((np.array([[]]), np.array([[]]), np.array([[]])))

    start = np.array([[]])
    end = np.array([[]])

    test_shape = to_utm(test_shape)
    for i in range(len(nogos)):
        nogos[i] = to_utm(nogos[i])
    start = to_utm(start)
    end = to_utm(end)

    # Calculate and print the bearing and distance between two points.
    target_loc = bearing_distance_from_coordinates(
        {
            EASTING: start[0, 0],
            NORTHING: start[0, 1],
            ELEVATION: 0
        }, {
            EASTING: end[0, 0],
            NORTHING: end[0, 1],
            ELEVATION: 0
        })

    print(target_loc)

    #### FIND OBJECTS ####
    mower = Robot(start[0, 0], start[0, 1], end)
    mower.lidar_range()
    centre_line = mower.lidar_range()[0][int(lidar_range / 2)]
    left_line = mower.lidar_range()[0][0]
    right_line = mower.lidar_range()[0][-1]

    gpd.GeoSeries(mower.lidar_range()[1]).plot()
    plt.plot(test_shape[:, 0], test_shape[:, 1])
    # plt.scatter(*detected_line.xy, color='red')
    plt.plot(*centre_line.xy)
    for nogo in nogos:
        plt.plot(nogo[:, 0], nogo[:, 1])
    plt.scatter(start[0, 0], start[0, 1])
    plt.scatter(mower.x, mower.y, color='blue')
    plt.scatter(end[0, 0], end[0, 1])
    plt.show()

    pos_bear = list(range(0, 90, 2))
    neg_bear = list(range(0, -90, -2))
    bear = [val for pair in zip(pos_bear, neg_bear) for val in pair]
    m = 0
    img = 0
    while abs(mower.x - end[0][0]) > 0.1 or abs(mower.y - end[0][1]) > 0.1:
        detected_line, img = get_detected_line(mower, end, nogos, 0,
                                               centre_line, test_shape, start,
                                               img)

        if detected_line == None:
            break
        centre_line = mower.lidar_range()[0][int(lidar_range / 2)]
        left_line = mower.lidar_range()[0][0]
        right_line = mower.lidar_range()[0][-1]
        while lidar_intersecting(centre_line, left_line, right_line, mower,
                                 detected_line):
            detected_line, img = get_detected_line(mower, end, nogos, bear[m],
                                                   centre_line, test_shape,
                                                   start, img)
            if detected_line == None:
                break
            centre_line = mower.lidar_range()[0][int(lidar_range / 2)]
            left_line = mower.lidar_range()[0][0]
            right_line = mower.lidar_range()[0][-1]
            m += 1
            print(m)

        mower.move(move_dist)
        mower.visited = np.vstack((mower.visited, [mower.x, mower.y]))
        gpd.GeoSeries(mower.lidar_range()[1]).plot()
        plt.plot(test_shape[:, 0], test_shape[:, 1])
        if detected_line != None:
            plt.scatter(detected_line.xy[0], detected_line.xy[1], color='red')
        plt.plot(mower.visited[:, 0], mower.visited[:, 1], color='blue')
        plt.plot(*centre_line.xy)
        plt.scatter(mower.x, mower.y, color='blue')
        for nogo in nogos:
            plt.plot(nogo[:, 0], nogo[:, 1])
        plt.scatter(start[0, 0], start[0, 1])
        plt.scatter(end[0, 0], end[0, 1])
        plt.savefig("look_position_" + str(img) + ".png")
        plt.clf()
        img += 1
        m = 0

    #### PLOT ####
    print(end)
    print([mower.x, mower.y])
    print(abs(mower.x - end[0][0]))
    print(abs(mower.y - end[0][1]))
    mower.visited = np.vstack((mower.visited, [mower.x, mower.y]))
    gpd.GeoSeries(mower.lidar_range()[1]).plot()
    plt.plot(test_shape[:, 0], test_shape[:, 1])
    # plt.scatter(*detected_line.xy, color='red')
    plt.plot(mower.visited[:, 0], mower.visited[:, 1], color='blue')
    plt.plot(*centre_line.xy)
    for nogo in nogos:
        plt.plot(nogo[:, 0], nogo[:, 1])
    plt.scatter(start[0, 0], start[0, 1])
    plt.scatter(end[0, 0], end[0, 1])
    plt.scatter(mower.x, mower.y, color='blue')
    plt.savefig("look_position_" + str(img) + ".png")
    # plt.show()


if __name__ == "__main__":
    main()
