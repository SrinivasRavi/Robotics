#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import random
import math
import sys
import tf

def find_line_model(x0,y0,x1,y1):
    m = (y1 - y0) / (x1 -x0 + sys.float_info.epsilon)  # slope (gradient) of the line
    c = y1 - m * x1                                     # y-intercept of the line
    return m, c

def find_intercept_point(m, c, x2, y2):
	x3 = (x2 + m*y2 - m*c)/(1 + m**2)
	y3 = (m*x2 + (m**2)*y2 - (m**2)*c)/(1 + m**2) + c
	return x3, y3

def broadcast_pose(msg):
    global goal_pos_x, goal_pos_y, robot_pos_x, robot_pos_y
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     'robot',
                     'world')
    br.sendTransform((goal_pos_x, goal_pos_y, 0),
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     'goal',
                     'world')
    robot_pos_x = msg.pose.pose.position.x
    robot_pos_y = msg.pose.pose.position.y

def set_robot_pose(base_pose_gt_msg): #not using
    global robot_pos_x, robot_pos_y, robot_angle
    qt = base_pose_gt_msg.pose.pose.orientation
    (_,_,robot_angle) = euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])
    robot_pos_x = base_pose_gt_msg.pose.pose.position.x
    robot_pos_y = base_pose_gt_msg.pose.pose.position.y
    print robot_angle

def base_scan_callback(laser_msg):
    global laser_ranges, laser_intensities
    laser_ranges = laser_msg.ranges
    laser_intensities = laser_msg.intensities

def follow_wall(laser_ranges, laser_intensities):
    if((laser_intensities[190:360]).count(1) < min_threshold): #too far away from the left wall
        forward_vel = 0
        rotational_vel = 50 #move left
        if(min(laser_ranges[181:360]) > 2.95): #lost contact to left wall. try to move towards a wall and turn right
            forward_vel = 2
            rotational_vel = 0 # when very near a wall, align in proper direction
    elif((laser_intensities[190:360]).count(1) > max_threshold): #too close to the left wall
        forward_vel = 0
        rotational_vel = -50 #move right
    else: #perfect. keep moving ahead
        forward_vel = 2
        rotational_vel = 0
    return forward_vel, rotational_vel

def follow_wall_properly(laser_ranges, laser_intensities):
    if((laser_intensities[190:360]).count(1) < min_threshold): #too far away from the left wall
        forward_vel = 0
        rotational_vel = 50 #move left
        if(min(laser_ranges[181:360]) > 2.95): #lost contact to left wall. try to move towards a wall and turn right
            forward_vel = 2
            rotational_vel = 0 # when very near a wall, align in proper direction
    elif((laser_intensities[190:360]).count(1) > max_threshold): #too close to the left wall
        forward_vel = 0
        rotational_vel = -50 #move right
    else: #perfect. keep moving ahead
        forward_vel = 2
        rotational_vel = 0
    return forward_vel, rotational_vel

def compute_goalseek_rot(goal_angle): #not using
    if(math.fabs(goal_angle) < math.pi/10):
        return 0
    else:
        return goal_angle * 100

def compute_goalseek_trans(ranges): #not using
    if len(ranges) > 0:
        min_laser_front = min(ranges[150:210])
        if(min_laser_front<1.9):
            return 0
        else:
            return min(2,min_laser_front-1)
    else:
        return 0

def obstacle_in_way(ranges):
    if(min(ranges[160:200])<0.6):
        return True
    else:
        return False

def crossed_line(m_line_m, m_line_c,robot_pos_x,robot_pos_y):
    global last_sign
    m_line_y = m_line_m * robot_pos_x + m_line_c
    if(robot_pos_y > m_line_y ):
        new_sign = 'pos'
    else:
        new_sign = 'neg'
    if new_sign == last_sign: #hasn't crossed the line
        return False
    else: #line crossing detected
        last_sign = new_sign
        return True

if __name__ == '__main__':
    rospy.init_node('bug2_demo', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    move = Twist()
    rospy.Subscriber('/base_scan',LaserScan,base_scan_callback)
    rospy.Subscriber('/base_pose_ground_truth', Odometry ,broadcast_pose)

    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    robot_state = 'GOALSEEK'
    robot_pos_x = -8
    robot_pos_y = -2
    robot_angle = 0
    goal_pos_x = 4.5#0#
    goal_pos_y = 9.0#-5#
    at_goal_threshold = 1.75
    at_goal = False
    min_threshold = 85
    max_threshold = 140

    last_sign = 'pos'
    laser_ranges = [3]*361
    laser_intensities = [0]*361
    trans = [5,5,0]

    rate = rospy.Rate(20)
    m_line_m, m_line_c = find_line_model(robot_pos_x,robot_pos_y,goal_pos_x,goal_pos_y)
    was_rotating = False
    
    while not rospy.is_shutdown():
        if(not at_goal):

            goal_dist = math.sqrt((goal_pos_x - robot_pos_x)**2 + (goal_pos_y - robot_pos_y)**2)
            #goal_angle = math.atan2((goal_pos_y - robot_pos_y), (goal_pos_x - robot_pos_x)) - robot_angle
            forward_vel = 0
            rotational_vel = 0
            if(goal_dist < at_goal_threshold):
                    forward_vel = 0
                    rotational_vel = 0
                    robot_state = 'ATDESTINATION'
                    at_goal = True
            else:
                if robot_state == 'GOALSEEK':
                    try: 
                        (trans,rot) = listener.lookupTransform('/robot', '/goal', rospy.Time(0)) 
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
                        pass
                    val = math.atan2(trans[1], trans[0])
                    if(val< -0.05 or val> 0.05): #math.atan2(trans[1], trans[0])**2
                        forward_vel = 0
                        rotational_vel = 0.5
                        was_rotating = True
                    else:
                        forward_vel = 2
                        rotational_vel = 0
                        was_rotating = False
                    if ((obstacle_in_way(laser_ranges)) and (not was_rotating)):
                        robot_state = 'WALLFOLLOW'
                        was_rotating = False
                        rotational_vel = -0.7
    
                elif robot_state == 'WALLFOLLOW':
                    forward_vel, rotational_vel = follow_wall(laser_ranges, laser_intensities)
                    if(crossed_line(m_line_m, m_line_c,robot_pos_x,robot_pos_y)):
                        robot_state = 'GOALSEEK'
                        was_rotating
                    
                move.linear.x = forward_vel
                move.angular.z = rotational_vel
                cmd_vel_pub.publish(move)
        rate.sleep()