#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import numpy as np
import shapely
from shapely.geometry import LineString, Polygon
from math import *
if __name__ == "__main__":
	import matplotlib.pyplot as plt
	import matplotlib.patches

def calc_path(poly, robot_width, bStartTop=False):
	"""
	Takes in a monotone shapely Polygon without obstacles and calculates a coverage path with given width through it
	"""
	ret = []
	up = False
	pos_next = None

	# Decrease width to integer number of runs
	poly_width = poly.bounds[2]-poly.bounds[0]
	poly_height = poly.bounds[3]-poly.bounds[1]

	if poly_width < robot_width and poly_height < robot_width:
		return ret # Don't bother

	# Reduce size of polygon by width
	poly_inner = poly.buffer(-robot_width/2.0)
	if robot_width >= poly_width or robot_width >= poly_height:
		# if smaller then width try at least one sweep
		xscale = 1-(robot_width/poly_width) if robot_width < poly_width else 0.0
		yscale = 1-(robot_width/poly_height) if robot_width < poly_height else 0.0
		poly_inner = shapely.affinity.scale(poly, xscale, yscale)

	plows = ceil(poly_width/robot_width)
	plow_width = poly_width/plows

	if poly_inner.is_empty:
		# No polygon left
		return ret

	for x in np.arange(poly_inner.bounds[0], poly_inner.bounds[2]+plow_width, plow_width):
		# Correct rounding errors
		if x > poly_inner.bounds[2]: x = poly_inner.bounds[2]

		# Get both intersections of current x with y
		intersections_y = None
		if poly_inner.bounds[0] == poly_inner.bounds[2]:
			# more a line then polygon
			intersections_y = [poly_inner.bounds[1], poly_inner.bounds[3]]
		else:
			try:
				vertline = LineString([(x, poly_inner.bounds[1]), (x, poly_inner.bounds[3])])
				intersections = poly_inner.intersection(vertline)
				if (type(intersections) != type(vertline)): continue
				intersections_y = [l[1] for l in intersections.coords[:]]
			except shapely.errors.TopologicalError:
				continue
		intersections_y.sort()
		if bStartTop: intersections_y.reverse()
		if len(intersections_y) < 2:
			# Only one intersection
			y1 = intersections_y[0]
			ret.append([x, y1])
		else:
			(y1, y2) = intersections_y 

			for i in range(2): # One to follow the current plow, one to drive to the next plow
				if i == 1:
					up = not up
				if up:
					pos_next = [x, y2]
				else:
					pos_next = [x, y1]

				pos_cur = pos_next
				if not pos_cur in ret:
					ret.append(pos_cur)

	return ret


if __name__ == "__main__":
	def paint_arrow(ax, pos_start, pos_end, width):
		length_x = pos_end[0]-pos_start[0]
		length_y = pos_end[1]-pos_start[1]
		ax.arrow(pos_start[0], pos_start[1], length_x, length_y, color='r', width=width/10, length_includes_head=True, shape='right')
		ax.arrow(pos_start[0], pos_start[1], length_x, length_y, color='r', width=width/10, length_includes_head=True, shape='left')
	
	def test(points, width=0.1, bStartTop=False):
		fig, ax = plt.subplots()
		poly = Polygon(points)
		path = calc_path(poly, width, bStartTop)

		pos_last = None
		for pos_cur in path:
			if pos_last is not None:
				paint_arrow(ax, pos_last, pos_cur, width)
			pos_last = pos_cur

		ax.axis([poly.bounds[0]-width*2, poly.bounds[2]+width*2, poly.bounds[1]-width*2, poly.bounds[3]+width*2])
		ax.add_patch(matplotlib.patches.Polygon(points))
		plt.gca().set_aspect('equal', adjustable='box')
		plt.show()

	test([[0.0, 0.3], [1.0, 0.1], [1.0, 1.5], [0.0, 1.2]])
	test([[0.0, 0.3], [1.0, 0.1], [1.0, 1.5], [0.0, 1.2]], bStartTop=True)
	test([[0.0, 0.3], [1.0, 0.3], [1.0, -1.2], [0.0, -1.2]])
	test([[99, 145], [101, 143], [102, 143], [107, 138], [108, 138], [112, 134], [113, 134], [117, 130], [118, 130], [122, 126], [123, 126], [127, 122], [128, 122], [132, 118], [133, 118], [137, 114], [138, 114], [142, 110], [143, 110], [147, 106], [148, 106], [152, 102], [153, 102], [157, 98], [158, 98], [162, 94], [163, 94], [167, 90], [168, 90], [172, 86], [173, 86], [177, 82], [178, 82], [181, 79], [183, 79], [185, 81], [186, 81], [190, 85], [191, 85], [195, 89], [196, 89], [201, 94], [202, 94], [206, 98], [207, 98], [211, 102], [212, 102], [216, 106], [217, 106], [221, 110], [222, 110], [226, 114], [227, 114], [231, 118], [232, 118], [236, 122], [237, 122], [241, 126], [242, 126], [246, 130], [247, 130], [251, 134], [252, 134], [256, 138], [257, 138], [261, 142], [262, 142], [265, 145], [265, 0], [99, 0]], 10.0)
	test([[355, 373], [501, 373], [501, 173], [500, 184], [499, 191], [498, 197], [497, 201], [496, 206], [495, 209], [494, 213], [491, 222], [490, 224], [489, 227], [487, 231], [486, 234], [483, 240], [482, 241], [480, 245], [479, 246], [478, 248], [477, 249], [476, 251], [475, 252], [474, 254], [471, 257], [470, 259], [463, 266], [462, 266], [458, 270], [457, 270], [455, 272], [454, 272], [453, 273], [452, 273], [450, 275], [448, 275], [447, 276], [446, 276], [445, 277], [444, 277], [443, 278], [440, 278], [439, 279], [435, 279], [434, 280], [423, 280], [422, 279], [417, 279], [416, 278], [413, 278], [412, 277], [411, 277], [410, 276], [409, 276], [408, 275], [406, 275], [404, 273], [403, 273], [402, 272], [401, 272], [399, 270], [398, 270], [394, 266], [393, 266], [387, 260], [386, 258], [383, 255], [382, 253], [380, 251], [379, 249], [378, 248], [377, 246], [376, 245], [373, 239], [372, 238], [371, 236], [370, 233], [367, 227], [365, 221], [364, 219], [363, 216], [362, 212], [361, 209], [359, 201], [358, 196], [356, 184], [355, 173]], 10.0)
	test([[1.0237047441031752, -0.7820379895883173], [-0.14663610049576714, -0.7820379895883174], [-0.08263086523790919, 0.08637301458856579], [0.9801839506405554, 0.07841088655608253], [1.003451938119474, -0.7755446380778018]])
	test([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], width=0.2)
	test([[0.0, 0.0], [0.25, 0.0], [0.25, 1.0], [0.0, 1.0]], width=0.2)
	test([[0.0, 0.0], [0.20, 0.0], [0.20, 1.0], [0.0, 1.0]], width=0.2)
	test([(0.11999988555908203, 1.7299998495727777), (0.1999998837709427, 1.7299998495727777), (0.20999988354742527, 1.7399998493492603), (0.21999988332390785, 1.7399998493492603), (0.22999988310039043, 1.7299998495727777), (0.2899998817592859, 1.7299998495727777), (0.2899998817592859, -1.960000067949295), (0.21999988332390785, -1.960000067949295), (0.20999988354742527, -1.9700000677257776), (0.1999998837709427, -1.960000067949295), (0.11999988555908203, -1.960000067949295), (0.11999988555908203, 1.7299998495727777)], width=0.3)
