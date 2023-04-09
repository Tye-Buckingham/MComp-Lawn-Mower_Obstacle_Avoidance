""" Small collection of functions to reduce code size when using
the surveytoolbox UTM functions.

Functions are not uniform, some use dicts, some return raw values.
# TODO make functions return predictable values
"""
import math

import numpy as np
import utm
from shapely.geometry import LineString, Point, Polygon
from surveytoolbox.bdc import bearing_distance_from_coordinates
from surveytoolbox.cbd import coordinates_from_bearing_distance
from surveytoolbox.config import EASTING, ELEVATION, NORTHING


def utm_bearing(p1, p2):
    """Short wrapper function to reduce lines of code.
    Returns the bearing of two points.
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


def utm_bearing_dist(p1, p2):
    """Short wrapper function to reduce lines of code.
    Returns the bearing and distance of two points.
    """
    bearing_dist = bearing_distance_from_coordinates(
        {
            EASTING: p1[0],
            NORTHING: p1[1],
            ELEVATION: 0
        }, {
            EASTING: p2[0],
            NORTHING: p2[1],
            ELEVATION: 0
        })
    return bearing_dist


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


def utm_dist(p1, p2):
    """Short wrapper function to reduce lines of code.
    Returns the distance from two points.
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


def utm_coords(point, heading, distance):
    """Short wrapper function to reduce lines of code.
    Returns the coordinates of a point from the current
    given a heading and distance.
    """
    coords = coordinates_from_bearing_distance(
        {
            EASTING: point[0],
            NORTHING: point[1],
            ELEVATION: 0
        }, heading, distance)
    return coords
