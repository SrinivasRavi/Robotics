#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from random import randint


def callback(msg):
	if ((min(msg.ranges[60:120])<2.4) or (min(msg.ranges[0:60])<1) or (min(msg.ranges[120:180])<1)):
		move.linear.x = 0
		move.angular.z = randint(0,359)
	else:
		move.linear.x = 2
		move.angular.z = 0
	pub.publish(move)

if __name__ == '__main__':
	rospy.init_node('move_and_evade_obstacles', anonymous=True)
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
	move = Twist()
	rospy.Subscriber('/base_scan',LaserScan,callback)

	rospy.spin()

