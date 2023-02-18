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
lidar_width = 15


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
            }, (360 - lidar_width) + self.heading, lidar_dist)
        right = coordinates_from_bearing_distance(
            {
                EASTING: self.x,
                NORTHING: self.y,
                ELEVATION: 0
            }, lidar_width + self.heading, lidar_dist)
        inter = []
        # interval = abs(((360 - 15) + self.heading) - (15 + self.heading)) / 120
        for i in range(lidar_range):
            pos = coordinates_from_bearing_distance(
                {
                    EASTING: self.x,
                    NORTHING: self.y,
                    ELEVATION: 0
                }, ((360 - lidar_width) + self.heading) + (i / 2), lidar_dist)

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

    def detect_objects(self, nogos, target):
        points = []
        lidar_lines = self.lidar_range()[0]
        for nogo in nogos[0]:
            for i in range(-1, len(nogo) - 1):
                edge = LineString([(nogo[i][0], nogo[i][1]),
                                   (nogo[i + 1][0], nogo[i + 1][1])])
                for line in lidar_lines:
                    if line.intersects(edge):
                        dist_from_intersect = bearing_distance_from_coordinates(
                            {
                                EASTING: self.x,
                                NORTHING: self.y,
                                ELEVATION: 0
                            }, {
                                EASTING: line.intersection(edge).x,
                                NORTHING: line.intersection(edge).y,
                                ELEVATION: 0
                            })
                        dist_from_target = bearing_distance_from_coordinates(
                            {
                                EASTING: self.x,
                                NORTHING: self.y,
                                ELEVATION: 0
                            }, {
                                EASTING: target[0],
                                NORTHING: target[1],
                                ELEVATION: 0
                            })

                        if dist_from_intersect['dist_2d'] > dist_from_target[
                                'dist_2d']:
                            continue
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

    def move(self, metres, target):
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
        print(target_loc)
        if self.heading == target_loc['bg'] and target_loc['dist_2d'] < 0.5:
            metres = (target_loc['dist_2d'] - 0.15)
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
                EASTING: end[0],
                NORTHING: end[1],
                ELEVATION: 0
            })
        self.heading = target_loc['bg']


def get_detected_line(mower, end, nogos, heading, centre_line, test_shape,
                      start, img, path):
    print(path[end])
    mower.find_target(path[end])
    objs = mower.detect_objects([nogos], path[end])
    if len(objs) != 0:
        mower.heading += heading
        objs = mower.detect_objects([nogos], path[end])
    # If no object in between target and robot, go straight to it
    while len(objs) == 0:
        mower.move(move_dist, path[end])
        mower.visited = np.vstack((mower.visited, [mower.x, mower.y]))
        gpd.GeoSeries(mower.lidar_range()[1]).plot()
        plt.plot(test_shape[:, 0], test_shape[:, 1])
        plt.plot(mower.visited[:, 0], mower.visited[:, 1], color='blue')
        centre_line = mower.lidar_range()[0][int(lidar_range / 2)]
        plt.plot(*centre_line.xy)
        plt.scatter(mower.x, mower.y, color='blue')
        for nogo in nogos:
            plt.plot(nogo[:, 0], nogo[:, 1])
        plt.scatter(path[start, 0], path[start, 1])
        plt.scatter(path[end, 0], path[end, 1])
        plt.axis('off')
        plt.savefig("./Imgs/look_position_" + str(img) + ".png")
        plt.clf()
        img += 1
        if abs(mower.x - path[end][0]) <= 0.20 and abs(mower.y -
                                                       path[end][1]) <= 0.20:
            return None, img
        mower.find_target(path[end])
        objs = mower.detect_objects([nogos], path[end])
        if len(objs) != 0:
            mower.heading += heading
            objs = mower.detect_objects([nogos], path[end])
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
    l = left.intersects(detected_line)
    r = right.intersects(detected_line)
    c = centre.intersects(detected_line)
    if c:
        return True, 0
    if l and r and not c:
        l_int = left.intersection(detected_line)
        r_int = right.intersection(detected_line)

        l_dist = bearing_distance_from_coordinates(
            {
                EASTING: mower.x,
                NORTHING: mower.y,
                ELEVATION: 0
            }, {
                EASTING: r_int.x,
                NORTHING: r_int.y,
                ELEVATION: 0
            })
        r_dist = bearing_distance_from_coordinates(
            {
                EASTING: mower.x,
                NORTHING: mower.y,
                ELEVATION: 0
            }, {
                EASTING: l_int.x,
                NORTHING: l_int.y,
                ELEVATION: 0
            })
        if l_dist['dist_2d'] < r_dist['dist_2d']:
            return True, -1
        else:
            return True, 1
    if l:
        return True, -1
    if r:
        return True, 1

    return False, None


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

    path = np.loadtxt("./route.out", dtype=float, delimiter=",")
    path_len = len(path)
    start = 0
    end = 1
    # path = to_utm(path)
    test_shape = to_utm(test_shape)
    for i in range(len(nogos)):
        nogos[i] = to_utm(nogos[i])

    # Calculate and print the bearing and distance between two points.
    target_loc = bearing_distance_from_coordinates(
        {
            EASTING: path[start, 0],
            NORTHING: path[start, 1],
            ELEVATION: 0
        }, {
            EASTING: path[end, 0],
            NORTHING: path[end, 1],
            ELEVATION: 0
        })

    print(target_loc)

    #### FIND OBJECTS ####
    mower = Robot(path[start, 0], path[start, 1], end)
    mower.visited = np.vstack((mower.visited, [path[0, 0], path[0, 1]]))

    mower.lidar_range()
    centre_lines = mower.lidar_range()[0][30]
    left_lines = mower.lidar_range()[0][0]
    right_lines = mower.lidar_range()[0][-1]

    gpd.GeoSeries(mower.lidar_range()[1]).plot()
    plt.plot(test_shape[:, 0], test_shape[:, 1])
    # plt.scatter(*detected_line.xy, color='red')
    centre_line = mower.lidar_range()[0][int(lidar_range / 2)]
    plt.plot(*centre_line.xy)
    for nogo in nogos:
        plt.plot(nogo[:, 0], nogo[:, 1])
    plt.scatter(path[start, 0], path[start, 1])
    plt.scatter(mower.x, mower.y, color='blue')
    plt.scatter(path[end, 0], path[end, 1])
    plt.show()

    right_bear = list(range(0, 180, 2))
    left_bear = list(range(0, -180, -2))
    centre_bear = [
        val for pair in zip(list(range(0, 180, 4)), list(range(0, -180, -4)))
        for val in pair
    ]
    m = 0
    img = 0
    while end < path_len:
        while abs(mower.x - path[end][0]) > 0.20 or abs(mower.y -
                                                        path[end][1]) > 0.20:
            detected_line, img = get_detected_line(mower, end, nogos, 0,
                                                   centre_line, test_shape,
                                                   start, img, path)

            if detected_line == None:
                break
            centre_lines = mower.lidar_range()[0][30]
            left_lines = mower.lidar_range()[0][0]
            right_lines = mower.lidar_range()[0][-1]
            lidar_intersect, side = lidar_intersecting(centre_lines,
                                                       left_lines, right_lines,
                                                       mower, detected_line)
            while lidar_intersect:
                if side == -1:
                    bear = right_bear
                elif side == 1:
                    bear = left_bear
                else:
                    bear = centre_bear

                detected_line, img = get_detected_line(mower, end, nogos,
                                                       bear[m], centre_line,
                                                       test_shape, start, img,
                                                       path)
                if detected_line == None:
                    start += 1
                    end += 1
                    break
                centre_lines = mower.lidar_range()[0][30]
                left_lines = mower.lidar_range()[0][0]
                right_lines = mower.lidar_range()[0][-1]
                m += 1
                print(m)
                if m == len(bear) - 1:
                    start += 1
                    end += 1
                    print(side)
                    print("Couldn't get to point")
                    return

            mower.move(move_dist, path[end])
            mower.visited = np.vstack((mower.visited, [mower.x, mower.y]))
            gpd.GeoSeries(mower.lidar_range()[1]).plot()
            plt.plot(test_shape[:, 0], test_shape[:, 1])
            if detected_line != None:
                plt.scatter(detected_line.xy[0],
                            detected_line.xy[1],
                            color='red')
            plt.plot(mower.visited[:, 0], mower.visited[:, 1], color='blue')
            plt.plot(*centre_line.xy)
            plt.scatter(mower.x, mower.y, color='blue')
            for nogo in nogos:
                plt.plot(nogo[:, 0], nogo[:, 1])
            plt.scatter(path[start, 0], path[start, 1])
            plt.scatter(path[end, 0], path[end, 1])
            plt.axis('off')
            plt.savefig("./Imgs/look_position_" + str(img) + ".png")
            plt.clf()
            img += 1
            m = 0
        start += 1
        end += 1

    #### PLOT ####
    mower.visited = np.vstack((mower.visited, [mower.x, mower.y]))
    gpd.GeoSeries(mower.lidar_range()[1]).plot()
    plt.plot(test_shape[:, 0], test_shape[:, 1])
    # plt.scatter(*detected_line.xy, color='red')
    plt.plot(mower.visited[:, 0], mower.visited[:, 1], color='blue')
    plt.plot(*centre_line.xy)
    for nogo in nogos:
        plt.plot(nogo[:, 0], nogo[:, 1])
    plt.scatter(path[start, 0], path[start, 1])
    plt.scatter(path[end, 0], path[end, 1])
    plt.scatter(mower.x, mower.y, color='blue')
    plt.axis('off')
    plt.savefig("./Imgs/look_position_" + str(img) + ".png")
    # plt.show()


if __name__ == "__main__":
    main()
