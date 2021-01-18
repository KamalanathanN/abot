#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import collections
from shapely.geometry import Polygon, Point
from shapely import affinity


def border_calc_path(poly, width, bStartTop=False):
	# Border path is polygon reduced by width
	poly_inner = poly.buffer(-width/2.0)

	(minx, miny, maxx, maxy) = poly_inner.bounds
	lower_left_edge = [minx, miny]
	top_left_edge = [minx, maxy]
	start_edge = Point(top_left_edge if bStartTop else lower_left_edge)

	# Find point of polygon closest to the start_edge
	points = poly_inner.exterior.coords[:]
	min_dist = float('inf')
	min_i = 0
	for i, point in enumerate(points):
		dist = Point(point).distance(start_edge)
		if dist < min_dist:
			min_dist = dist
			min_i = i

	# last point is start point, remove it before rotation
	points = points[:-1]
	coll = collections.deque(points)
	# Rotate points of polygon to begin start_edge
	coll.rotate(-min_i)
	# append last point again
	coll.append(coll[0])
	
	return list(coll)


if __name__ == "__main__":
	import matplotlib.pyplot as plt
	import matplotlib.patches

	def paint_arrow(ax, pos_start, pos_end, width):
		length_x = pos_end[0]-pos_start[0]
		length_y = pos_end[1]-pos_start[1]
		ax.arrow(pos_start[0], pos_start[1], length_x, length_y, color='r', width=width/10, length_includes_head=True, shape='right')
		ax.arrow(pos_start[0], pos_start[1], length_x, length_y, color='r', width=width/10, length_includes_head=True, shape='left')
	
	def test(points, width=0.1, bStartTop=False):
		fig, ax = plt.subplots()
		poly = Polygon(points)
		path = border_calc_path(poly, width, bStartTop)

		pos_last = None
		for pos_cur in path:
			if pos_last is not None:
				paint_arrow(ax, pos_last, pos_cur, width)
			pos_last = pos_cur

		ax.axis([poly.bounds[0]-width*2, poly.bounds[2]+width*2, poly.bounds[1]-width*2, poly.bounds[3]+width*2])
		ax.add_patch(matplotlib.patches.Polygon(points))
		plt.gca().set_aspect('equal', adjustable='box')
		plt.show()
		return path

	assert test([[0.0, 0.3], [1.0, 0.1], [1.0, 1.5], [0.0, 1.2]], bStartTop=False) == [(0.05, 0.3409901951359278), (0.05, 1.1627984674554472), (0.95, 1.4327984674554473), (0.95, 0.16099019513592785), (0.05, 0.3409901951359278)]
	assert test([[0.0, 0.3], [1.0, 0.1], [1.0, 1.5], [0.0, 1.2]], bStartTop=True) == [(0.05, 1.1627984674554472), (0.95, 1.4327984674554473), (0.95, 0.16099019513592785), (0.05, 0.3409901951359278), (0.05, 1.1627984674554472)]
	test([[99, 145], [101, 143], [102, 143], [107, 138], [108, 138], [112, 134], [113, 134], [117, 130], [118, 130], [122, 126], [123, 126], [127, 122], [128, 122], [132, 118], [133, 118], [137, 114], [138, 114], [142, 110], [143, 110], [147, 106], [148, 106], [152, 102], [153, 102], [157, 98], [158, 98], [162, 94], [163, 94], [167, 90], [168, 90], [172, 86], [173, 86], [177, 82], [178, 82], [181, 79], [183, 79], [185, 81], [186, 81], [190, 85], [191, 85], [195, 89], [196, 89], [201, 94], [202, 94], [206, 98], [207, 98], [211, 102], [212, 102], [216, 106], [217, 106], [221, 110], [222, 110], [226, 114], [227, 114], [231, 118], [232, 118], [236, 122], [237, 122], [241, 126], [242, 126], [246, 130], [247, 130], [251, 134], [252, 134], [256, 138], [257, 138], [261, 142], [262, 142], [265, 145], [265, 0], [99, 0]], 10.0)
