#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import tf	

def broadcast_robot_pose(msg):
	br = tf.TransformBroadcaster()
	br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     'robot_1',
                     'world')

if __name__ == '__main__':
	rospy.init_node('pursue_other_robot', anonymous=True)

	listener = tf.TransformListener()
	robot_vel = rospy.Publisher('/robot_1/cmd_vel', Twist,queue_size=1)
	cmd = Twist()
	rate = rospy.Rate(10.0)

	rospy.Subscriber('/robot_1/base_pose_ground_truth', Odometry ,broadcast_robot_pose)
	#ideally should use /robot_0/odom but that makes the robot not follow properly
	while not rospy.is_shutdown():
		try:
		    (trans,rot) = listener.lookupTransform('/robot_1', '/robot_0', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		    continue
		
		angular = 4 * math.atan2(trans[1], trans[0])
		distance_bw_robots = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
		linear = 0.5 * distance_bw_robots
		if(distance_bw_robots>1.5):
			cmd.linear.x = linear
			cmd.angular.z = angular
		else:
			cmd.linear.x = 0
			cmd.angular.z = 0
		robot_vel.publish(cmd)

		rate.sleep()

