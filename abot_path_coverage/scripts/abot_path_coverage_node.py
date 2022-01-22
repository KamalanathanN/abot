#!/usr/bin/env python3
# -*- coding: iso-8859-15 -*-

import os
import rospy
import rospkg
import tf
import actionlib
import numpy as np
import pdb
import json
import tempfile
import tf2_ros
import tf2_geometry_msgs
from libs.list_helper import *
from libs.marker_visualization import MarkerVisualization
from shapely.geometry import Polygon, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from math import *
from libs.trapezoidal_coverage import calc_path as trapezoid_calc_path
from libs.border_drive import border_calc_path


INSCRIBED_INFLATED_OBSTACLE = 253

class MapDrive(MarkerVisualization):
	def __init__(self):
		rospy.init_node('map_drive')
		self.rospack = rospkg.RosPack()
		MarkerVisualization.__init__(self)

		self.lClickPoints = []
		self.global_frame = "map" # read from "/clicked_point"
		self.local_costmap = None
		self.global_costmap = None
		self.robot_width = rospy.get_param("~robot_width", 0.3)
		self.costmap_max_non_lethal = rospy.get_param("~costmap_max_non_lethal", 70)
		self.boustrophedon_decomposition = rospy.get_param("~boustrophedon_decomposition", True)
		self.border_drive = rospy.get_param("~border_drive", False)
		self.base_frame = rospy.get_param("~base_frame", "base_link")

		rospy.Subscriber("/clicked_point", PointStamped, self.rvizPointReceived)
		rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.globalCostmapReceived)
		rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.localCostmapReceived)
		self.tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(self.tfBuffer)
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		rospy.loginfo("Waiting for the move_base action server to come up")
		self.move_base.wait_for_server()
		self.move_base_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
		rospy.loginfo("Got move_base action server")
		rospy.on_shutdown(self.on_shutdown)

		rospy.loginfo("Running..")

	def globalCostmapReceived(self, costmap):
		self.global_costmap = costmap

	def localCostmapReceived(self, costmap):
		self.local_costmap = costmap
		self.local_costmap_width = costmap.info.width*costmap.info.resolution
		self.local_costmap_height = costmap.info.height*costmap.info.resolution

	def rvizPointReceived(self, point):
		self.lClickPoints.append(point)
		points = [(p.point.x, p.point.y) for p in self.lClickPoints]
		self.global_frame = point.header.frame_id
		if len(self.lClickPoints) > 2:
			# All points must have same frame_id
			if len(set([p.header.frame_id for p in self.lClickPoints])) != 1:
				raise
			points_x = [p.point.x for p in self.lClickPoints]
			points_y = [p.point.y for p in self.lClickPoints]
			avg_x_dist = list_avg_dist(points_x)
			avg_y_dist = list_avg_dist(points_y)
			dist_x_first_last = abs(points_x[0] - points_x[-1])
			dist_y_first_last = abs(points_y[0] - points_y[-1])
			if dist_x_first_last < avg_x_dist/10.0 and dist_y_first_last < avg_y_dist/10.0:
				# last point is close to maximum, construct polygon
				rospy.loginfo("Creating polygon %s" % (str(points)))
				self.visualize_area(points, close=True)
				if self.boustrophedon_decomposition:
					self.do_boustrophedon(Polygon(points), self.global_costmap)
				else:
					self.drive_polygon(Polygon(points))
				self.visualize_area(points, close=True, show=False)
				self.lClickPoints = []
				return
		self.visualize_area(points, close=False)


	def do_boustrophedon(self, poly, costmap):
		# Cut polygon area from costmap
		(minx, miny, maxx, maxy) = poly.bounds
		rospy.loginfo("Converting costmap at x=%.2f..%.2f, y=%.2f %.2f for Boustrophedon Decomposition" % (minx, maxx, miny, maxy))

		# Convert to costmap coordinate
		minx = round((minx-costmap.info.origin.position.x)/costmap.info.resolution)
		maxx = round((maxx-costmap.info.origin.position.x)/costmap.info.resolution)
		miny = round((miny-costmap.info.origin.position.y)/costmap.info.resolution)
		maxy = round((maxy-costmap.info.origin.position.y)/costmap.info.resolution)

		# Check min/max limits
		if minx < 0: minx = 0
		if maxx > costmap.info.width: maxx = costmap.info.width
		if miny < 0: miny = 0
		if maxy > costmap.info.height: maxy = costmap.info.height
		
		# Transform costmap values to values expected by boustrophedon_decomposition script
		rows = []
		for ix in range(int(minx), int(maxx)):
			column = []
			for iy in range(int(miny), int(maxy)):
				x = ix*costmap.info.resolution+costmap.info.origin.position.x
				y = iy*costmap.info.resolution+costmap.info.origin.position.y
				data = costmap.data[int(iy*costmap.info.width+ix)]
				if data == -1 or not poly.contains(Point([x,y])):
					# Unknown or not inside polygon: Treat as obstacle
					column.append(0)
				elif data <= self.costmap_max_non_lethal:
					# Freespace (non-lethal)
					column.append(-1)
				else:
					# Obstacle
					column.append(0)
			rows.append(column)
		#pdb.set_trace()
		polygons = []
		with tempfile.NamedTemporaryFile(delete=False,mode='w') as ftmp:
			ftmp.write(json.dumps(rows))
			ftmp.flush()
			boustrophedon_script = os.path.join(self.rospack.get_path('abot_path_coverage'), "scripts/boustrophedon_decomposition.rb")
			with os.popen("%s %s" % (boustrophedon_script, ftmp.name)) as fscript:
				polygons = json.loads(fscript.readline())
		for poly in polygons:
			points = [
					(
					(point[0]+minx)*costmap.info.resolution+costmap.info.origin.position.x,
					(point[1]+miny)*costmap.info.resolution+costmap.info.origin.position.y
					) for point in poly]
			rospy.logdebug("Creating polygon from Boustrophedon Decomposition %s" % (str(points)))
			self.drive_polygon(Polygon(points))
		rospy.loginfo("Boustrophedon Decomposition done")

	def next_pos(self, x, y, angle):
		rospy.loginfo("Moving to (%f, %f, %.0f�)" % (x, y, angle*180/pi))

		goal = MoveBaseGoal()
		angle_quat = tf.transformations.quaternion_from_euler(0, 0, angle)
		goal.target_pose.header.frame_id = self.global_frame
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.orientation.x = angle_quat[0]
		goal.target_pose.pose.orientation.y = angle_quat[1]
		goal.target_pose.pose.orientation.z = angle_quat[2]
		goal.target_pose.pose.orientation.w = angle_quat[3]
		self.move_base.send_goal(goal)

		self.move_base.wait_for_result()

		if self.move_base.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo("The base moved to (%f, %f)" % (x, y))
		else:
			rospy.logerr("The base failed moving to (%f, %f)" % (x, y))

	def on_shutdown(self):
		rospy.loginfo("Canceling all goals")
		self.visualization_cleanup()
		self.move_base.cancel_all_goals()

	def get_closes_possible_goal(self, pos_last, pos_next, angle, tolerance):
		angle_quat = tf.transformations.quaternion_from_euler(0, 0, angle)
		start = PoseStamped()
		start.header.frame_id = self.global_frame
		start.pose.position.x = pos_last[0]
		start.pose.position.y = pos_last[1]
		start.pose.orientation.x = angle_quat[0]
		start.pose.orientation.y = angle_quat[1]
		start.pose.orientation.z = angle_quat[2]
		start.pose.orientation.w = angle_quat[3]
		goal = PoseStamped()
		goal.header.frame_id = self.global_frame
		goal.pose.position.x = pos_next[0]
		goal.pose.position.y = pos_next[1]
		goal.pose.orientation.x = angle_quat[0]
		goal.pose.orientation.y = angle_quat[1]
		goal.pose.orientation.z = angle_quat[2]
		goal.pose.orientation.w = angle_quat[3]
		plan = self.move_base_plan(start, goal, tolerance).plan
		if len(plan.poses) == 0:
			return None
		#pdb.set_trace()
		closest = None
		for pose in plan.poses:
			pose.header.stamp = rospy.Time(0) # time for lookup does not need to be exact since we are stopped

			local_pose = self.tfBuffer.transform(pose, self.local_costmap.header.frame_id)

			cellx = round((local_pose.pose.position.x-self.local_costmap.info.origin.position.x)/self.local_costmap.info.resolution)
			celly = round((local_pose.pose.position.y-self.local_costmap.info.origin.position.y)/self.local_costmap.info.resolution)
			cellidx = int(celly*self.local_costmap.info.width+cellx)
			if cellidx < 0 or cellidx >= len(self.local_costmap.data):
				rospy.logwarn("get_closes_possible_goal landed outside costmap, returning original goal.")
				return pos_next
			cost = self.local_costmap.data[cellidx]

			if (cost >= INSCRIBED_INFLATED_OBSTACLE):
				break

			closest = pose
		return (closest.pose.position.x, closest.pose.position.y)

	def drive_path(self, path):
		self.visualize_path(path)

		initial_pos = self.tfBuffer.lookup_transform(self.global_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.0))
		path.insert(0, (initial_pos.transform.translation.x, initial_pos.transform.translation.y))

		for pos_last,pos_next in pairwise(path):
			if rospy.is_shutdown(): return

			pos_diff = np.array(pos_next)-np.array(pos_last)
			# angle from last to current position
			angle = atan2(pos_diff[1], pos_diff[0])

			if abs(pos_diff[0]) < self.local_costmap_width/2.0 and abs(pos_diff[1]) < self.local_costmap_height/2.0:
				# goal is visible in local costmap, check path is clear
				tolerance = min(pos_diff[0], pos_diff[1])
				closest = self.get_closes_possible_goal(pos_last, pos_next, angle, tolerance)
				if closest is None:
					continue
				pos_next = closest

			self.next_pos(pos_last[0], pos_last[1], angle) # rotate in direction of next goal
			self.next_pos(pos_next[0], pos_next[1], angle)
		self.visualize_path(path, False)
	
	def drive_polygon(self, polygon):
		self.visualize_cell(polygon.exterior.coords[:])

		# Align longest side of the polygon to the horizontal axis
		angle = get_angle_of_longest_side_to_horizontal(polygon)
		if angle == None:
			rospy.logwarn("Can not return polygon")
			return
		angle+=pi/2 # up/down instead of left/right
		poly_rotated = rotate_polygon(polygon, angle)
		rospy.logdebug("Rotated polygon by %.0f�: %s" % (angle*180/pi, str(poly_rotated.exterior.coords[:])))

		if self.border_drive:
			path_rotated = border_calc_path(poly_rotated, self.robot_width)
			path = rotate_points(path_rotated, -angle)
			self.drive_path(path)

		# run
		path_rotated = trapezoid_calc_path(poly_rotated, self.robot_width)
		path = rotate_points(path_rotated, -angle)
		self.drive_path(path)

		# cleanup
		self.visualize_cell(polygon.exterior.coords[:], False)
		rospy.logdebug("Polygon done")


if __name__ == "__main__":
	p = MapDrive()
	rospy.spin()
