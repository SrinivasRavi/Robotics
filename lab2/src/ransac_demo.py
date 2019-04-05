#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import sys
import random
import math
#import time

def find_line_model(x0,y0,x1,y1):
    m = (y1 - y0) / (x1 -x0 + sys.float_info.epsilon)  # slope (gradient) of the line
    c = y1 - m * x1                                     # y-intercept of the line
    return m, c

def find_intercept_point(m, c, x2, y2):
	x3 = (x2 + m*y2 - m*c)/(1 + m**2)
	y3 = (m*x2 + (m**2)*y2 - (m**2)*c)/(1 + m**2) + c
	return x3, y3

def base_scan_callback(laser_msg):
	laser_beams_ranges = []
	global loop_control_count
	for i in range(len(laser_msg.ranges)):
		if(laser_msg.ranges[i]<3):#save only the ones detecting objects
			p = Point()
			p.x = laser_msg.ranges[i] * math.cos(((i-180)*math.pi)/360);
			p.y = laser_msg.ranges[i] * math.sin(((i-180)*math.pi)/360);
			p.z = 0 #keep it 0 for now. This does not matter anyways
			laser_beams_ranges.append(p)
	
	ransac_lines = []
	points_left = laser_beams_ranges
	loop_control_count += 1
	while(len(points_left)>point_left_count and (loop_control_count%1==0)):#Loop till most points have been removed
		max_inlier_count = 0
		final_inlier_outlier_list = []
		p0 = Point()
		p1 = Point()
		p0.z = 0
		p1.z = 0
		for k in range(iterations):#Loop for k iterations
			#pick 2 random points
			randomly_picked = random.sample(range(0,len(points_left)),2)
			x0 = points_left[randomly_picked[0]].x
			y0 = points_left[randomly_picked[0]].y
			x1 = points_left[randomly_picked[1]].x
			y1 = points_left[randomly_picked[1]].y
			[m,c]=find_line_model(x0,y0,x1,y1)
			current_inlier_count = 0
			inlier_outlier_list = [1] * len(points_left)

			for l in range(len(points_left)):
				#for each point x2,y2 in points_left calculate distance
				x2 = points_left[l].x
				y2 = points_left[l].y
				x3, y3 = find_intercept_point(m, c, x2, y2)
				distance = math.sqrt((x3 - x2)**2 + (y3 - y2)**2)
				if(distance<distance_threshold):
					inlier_outlier_list[l]=0 #set as inlier
					current_inlier_count += 1
				else:
					inlier_outlier_list[l]=1 #set as outlier

			#calculate inliers and outliers
			if(max_inlier_count<current_inlier_count):
				max_inlier_count = current_inlier_count
				final_inlier_outlier_list = inlier_outlier_list
				# p0.x = x0
				# p0.y = y0
				# p1.x = x1
				# p1.y = y1

		# ransac_lines.append(p0)
		# ransac_lines.append(p1)

		new_points_left = []
		max_dist = 0
		for m in range(len(points_left)):
			if(final_inlier_outlier_list[m] == 1):
				new_points_left.append(points_left[m])
			else:
				for n in range(len(points_left)):
					if(final_inlier_outlier_list[n] == 0):
						curr_dist = math.sqrt((points_left[m].x - points_left[n].x)**2 + (points_left[m].y - points_left[n].y)**2)
						if max_dist < curr_dist:
							max_dist = curr_dist
							p0.x = points_left[m].x
							p0.y = points_left[m].y
							p1.x = points_left[n].x
							p1.y = points_left[n].y
		points_left = new_points_left
		ransac_lines.append(p0)
		ransac_lines.append(p1)
	
	lines_marker.points = ransac_lines
	marker_pub.publish(lines_marker)


if __name__ == '__main__':
	rospy.init_node('ransac_demo', anonymous=True)
	
	marker_pub = rospy.Publisher('/visualization_marker',Marker,queue_size=10)
	
	loop_control_count = 0
	#Set up line marker
	lines_marker = Marker()
	lines_marker.header.frame_id = '/base_laser_link'
	lines_marker.ns = 'lines'
	lines_marker.action = lines_marker.ADD
	lines_marker.type = lines_marker.LINE_LIST
	lines_marker.pose.orientation.w = 1
	lines_marker.id = 1
	lines_marker.scale.x = 0.05
	lines_marker.color.r = 1.0
	lines_marker.color.a = 1.0	
	
	#RANSAC parameters
	point_left_count = 5 #don't make this lesser than 2
	distance_threshold = 0.1
	iterations = 20

	rospy.Subscriber('/base_scan',LaserScan,base_scan_callback)
	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.2)

