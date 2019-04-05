#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from random import randint
import math
import tf

def broadcast_robot_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     'robot_0',
                     'world')

def callback(msg):
	## The change to avoid the pursuer
	try: 
		(trans,rot) = listener.lookupTransform('/robot_0', '/robot_1', rospy.Time(0)) 
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
		pass

	orientation = (4 * math.atan2(trans[1], trans[0]))**2 
	
	if ((min(msg.ranges[60:120])<2.4) or (min(msg.ranges[0:60])<1) or (min(msg.ranges[120:180])<1) or (math.sqrt(trans[0] ** 2 + trans[1] ** 2)<1 and orientation<15)): ##	
		move.linear.x = 0
		move.angular.z = randint(0,359)
	else:
		move.linear.x = 2
		move.angular.z = 0
	pub.publish(move)

if __name__ == '__main__':
	rospy.init_node('move_and_evade_obstacles', anonymous=True)
	pub = rospy.Publisher('/robot_0/cmd_vel',Twist,queue_size=1)
	move = Twist()

	listener = tf.TransformListener()
	rospy.Subscriber('/robot_0/base_scan',LaserScan,callback)

	rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry ,broadcast_robot_pose)
	#ideally should use /robot_0/odom but that makes the follower not follow properly
	rospy.spin()

