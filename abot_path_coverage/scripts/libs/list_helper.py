#!/usr/bin/env python3
# -*- coding: iso-8859-15 -*-

import itertools
import numpy as np
from shapely.affinity import affine_transform
from shapely.geometry import Polygon
from math import *
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import matplotlib.patches


# From https://docs.python.org/2.7/library/itertools.html#recipes
def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


def list_avg_dist(l):
    """Returns the average distance between values in given list"""
    dists = [abs(combo[0]-combo[1]) for combo in pairwise(l)]
    return sum(dists)/len(dists)


def rotate_points(points, angle):
    rotm = np.matrix([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
    return [(point*rotm).tolist()[0] for point in points]


def rotate_polygon(poly, angle):
    coords = poly.exterior.coords[:]
    points_rotated = rotate_points(coords, angle)
    return Polygon(points_rotated)


def get_angle_of_longest_side_to_horizontal(poly):
    """
    Returns the angle needed to align the longest side
    of the polygon to the horizontal axis
    """
    rect = poly.minimum_rotated_rectangle
    if not hasattr(rect, "exterior"):
        # is probably a point
        return None
    rect_points = rect.exterior.coords[:]
    p1,p2,p3,p4 = [np.array(p) for p in rect_points[:-1]]
    p12 = p1-p2
    p23 = p2-p3
    longest_side = p12 if np.linalg.norm(p12) > np.linalg.norm(p23) else p23
    return atan2(longest_side[1], longest_side[0])


# From shapely version 1.6
@property
def minimum_rotated_rectangle(self):
    """Returns the general minimum bounding rectangle of
    the geometry. Can possibly be rotated. If the convex hull
    of the object is a degenerate (line or point) this same degenerate
    is returned.
    """
    # first compute the convex hull
    hull = self.convex_hull
    try:
        coords = hull.exterior.coords
    except AttributeError:  # may be a Point or a LineString
        return hull
    # generate the edge vectors between the convex hull's coords
    edges = ((pt2[0] - pt1[0], pt2[1] - pt1[1]) for pt1, pt2 in zip(
        coords, itertools.islice(coords, 1, None)))

    def _transformed_rects():
        for dx, dy in edges:
        # compute the normalized direction vector of the edge
        # vector.
            length = sqrt(dx**2+dy**2)
        ux, uy = dx / length, dy / length
        # compute the normalized perpendicular vector
        vx, vy = -uy, ux
        # transform hull from the original coordinate system to
        # the coordinate system defined by the edge and compute
        # the axes-parallel bounding rectangle.
        transf_rect = affine_transform(
            hull, (ux, uy, vx, vy, 0, 0)).envelope
        # yield the transformed rectangle and a matrix to
        # transform it back to the original coordinate system.
        yield (transf_rect, (ux, vx, uy, vy, 0, 0))

    # check for the minimum area rectangle and return it
    transf_rect, inv_matrix = min(
        _transformed_rects(), key=lambda r: r[0].area)
    return affine_transform(transf_rect, inv_matrix)
if not hasattr(Polygon, "minimum_rotated_rectangle"):
    Polygon.minimum_rotated_rectangle = minimum_rotated_rectangle

if __name__ == "__main__":
    fig, ax = plt.subplots()
    points = [(1.58, 0.44), (0.78, -0.56), (0.38, -0.09), (1.00, 0.80), (1.57, 0.43)]
    
    ax.axis([-2, 2, -2, 2])

    ax.add_patch(matplotlib.patches.Polygon(points))

    angle = get_angle_of_longest_side_to_horizontal(Polygon(points))
    points2 = rotate_points(points, angle)

    ax.add_patch(matplotlib.patches.Polygon(points2, color='r'))

    plt.show()
