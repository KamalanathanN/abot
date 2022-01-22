#!/usr/bin/env python3
# -*- coding: iso-8859-15 -*-

import rospy
#from list_helper import *
from libs.list_helper import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerVisualization:
	def __init__(self):
		self.last_points = {}
		self.last_path = None
		self.pub_marker = rospy.Publisher("path_coverage_marker", Marker, queue_size=16)
	
	def visualization_cleanup(self):
		for id, points in self.last_points.items():
			if points is not None:
				self.visualize_trapezoid(points, id=id, show=False)
			self.last_points = {}
		if self.last_path is not None:
			self.visualize_path(self.last_path, False)
			self.last_path = None

	def visualize_cell(self, points, show=True, close=True):
		self.visualize_trapezoid(points, show, close)

	def visualize_area(self, points, show=True, close=True):
		self.visualize_trapezoid(points, show, close, id=1, red=1.0, blue=0.0)

	def visualize_trapezoid(self, points, show=True, close=True, id=0, red=0.0, green=0.0, blue=1.0):
		if len(points) < 2: return

		self.last_points[id] = points if show else None

		msg = Marker()
		msg.header.frame_id = self.global_frame
		msg.header.stamp = rospy.Time.now()
		msg.ns = "trapezoid"
		msg.lifetime = rospy.Duration(0)
		msg.id = id
		msg.type = Marker.LINE_STRIP
		msg.action = Marker.ADD if show else Marker.DELETE
		msg.pose.orientation.w = 1
		msg.pose.orientation.x = 0
		msg.pose.orientation.y = 0
		msg.pose.orientation.z = 0
		msg.scale.x = 0.02;
		# blue
		msg.color.r = red;
		msg.color.g = green;
		msg.color.b = blue;
		msg.color.a = 1.0;
		if close:
			points = points + [points[0]]
		for point in points:
			point_msg = Point()
			point_msg.x = point[0]
			point_msg.y = point[1]
			msg.points.append(point_msg)
		self.pub_marker.publish(msg)

	def visualize_path(self, path, show=True):
		i = 0
		self.last_path = path if show else None
		for pos_last,pos_cur in pairwise(path):
			msg = Marker()
			msg.header.frame_id = self.global_frame
			msg.header.stamp = rospy.Time.now()
			msg.ns = "path"
			msg.lifetime = rospy.Duration(0)
			msg.id = i
			msg.type = Marker.ARROW
			msg.action = Marker.ADD if show else Marker.DELETE
			msg.pose.orientation.w = 1
			msg.pose.orientation.x = 0
			msg.pose.orientation.y = 0
			msg.pose.orientation.z = 0
			msg.scale.x = 0.01; # shaft diameter
			msg.scale.y = 0.03; # head diameter
			# green
			msg.color.g = 1.0;
			msg.color.a = 1.0;

			point_msg_start = Point()
			point_msg_start.x = pos_last[0]
			point_msg_start.y = pos_last[1]
			msg.points.append(point_msg_start)
			point_msg_end = Point()
			point_msg_end.x = pos_cur[0]
			point_msg_end.y = pos_cur[1]
			msg.points.append(point_msg_end)

			i+=1
			self.pub_marker.publish(msg)
