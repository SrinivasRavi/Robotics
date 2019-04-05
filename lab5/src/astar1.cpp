/*
Program: A* search algorithm implementation for Stage ROS turtlebot
Author: Srinivas Ravi
Reference taken from:
1. https ://www.redblobgames.com/pathfinding/a-star/  Copyright 2014 Red Blob Games <redblobgames@gmail.com> : For concept and pseudocode
2. http://www.cplusplus.com/reference/queue/priority_queue/priority_queue/ : For priority queue min heap syntax

*/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h> 

#include<iostream>
#include <stdio.h>
#include <array>
#include <map>
#include <queue>
#include <vector>
#include <math.h>
#include <list>

//A-star logic specific initializations
std::array<int, 8> x_offset = { -1, 0, 1, 1, 1, 0, -1, -1 };
std::array<int, 8> y_offset = { 1, 1, 1, 0, -1, -1, -1, 0 };
std::array<double, 8> neighbor_cost = { 1.414, 1, 1.414, 1, 1.414, 1, 1.414, 1 };

struct loc_real
{
    double x_real;
    double y_real;
};

std::deque<loc_real> shortest_path, bkp_shortest_path;

struct loc_matrix
{
    int x_matrix;
    int y_matrix;
};

bool operator==(const loc_matrix& lhs, const loc_matrix& rhs)
{
    return ((lhs.x_matrix == rhs.x_matrix) && (lhs.y_matrix == rhs.y_matrix));
}
/*
loc_matrix real_to_matrix(loc_real given_loc_real)
{
    loc_matrix corr_loc_matrix;
    corr_loc_matrix.x_matrix = (int)(given_loc_real.x_real + 8);
    corr_loc_matrix.y_matrix = (int)(9 - given_loc_real.y_real);
    return corr_loc_matrix;
}

loc_real matrix_to_real(loc_matrix given_loc_matrix)
{
    loc_real corr_loc_real;
    corr_loc_real.x_real = (double)(given_loc_matrix.x_matrix - 8);
    corr_loc_real.y_real = (double)(9 - given_loc_matrix.y_matrix);
    return corr_loc_real;
}
*/

loc_matrix real_to_matrix(loc_real given_loc_real)
{
    loc_matrix corr_loc_matrix;
    corr_loc_matrix.x_matrix = int(floor(given_loc_real.x_real) + 9);
    corr_loc_matrix.y_matrix = int(10 - floor(given_loc_real.y_real));
    return corr_loc_matrix;
}

loc_real matrix_to_real(loc_matrix given_loc_matrix)
{
    loc_real corr_loc_real;
    corr_loc_real.x_real = given_loc_matrix.x_matrix - 8.5;
    corr_loc_real.y_real = 10.5 - given_loc_matrix.y_matrix;
    return corr_loc_real;
}

double heuristic(loc_matrix l1, loc_matrix l2) {
    return sqrt(pow((l2.x_matrix - l1.x_matrix), 2) + pow((l2.y_matrix - l1.y_matrix), 2));
}

bool is_valid_loc_matrix(loc_matrix given_loc_matrix){
    if (0 <= given_loc_matrix.x_matrix && given_loc_matrix.x_matrix < 18 && 0 <= given_loc_matrix.y_matrix && given_loc_matrix.y_matrix < 20)
        return true;
    else
        return false;
}

struct priority_loc_matrix {
    double priority;
    loc_matrix location_matrix;

    priority_loc_matrix(double priority1, loc_matrix location_matrix1) {
        priority = priority1;
        location_matrix = location_matrix1;
    }
};

bool operator<(const priority_loc_matrix& lhs, const priority_loc_matrix& rhs)
{
    return lhs.priority > rhs.priority;
}


void a_star_search(std::array<std::array<int, 20>, 18> map_matrix, loc_matrix start_matrix, loc_matrix goal_matrix, std::array<std::array<loc_matrix, 20>, 18>& came_from, std::array<std::array<double, 20>, 18>& cost_so_far) {
    std::priority_queue<priority_loc_matrix> frontier;
    frontier.emplace(0, start_matrix);

    came_from[start_matrix.x_matrix][start_matrix.y_matrix] = start_matrix;
    cost_so_far[start_matrix.x_matrix][start_matrix.y_matrix] = 0.0;

    while (!frontier.empty())
    {
        loc_matrix current_matrix = frontier.top().location_matrix;
        frontier.pop();
        if (current_matrix == goal_matrix)
        {
            break;
        }

        std::array<loc_matrix, 8> neighbors;
        std::array<bool, 8> validity_neighbors;
        for (int i = 0; i < 8; i++) {
            loc_matrix neighbor;
            neighbor.x_matrix = current_matrix.x_matrix + x_offset[i];
            neighbor.y_matrix = current_matrix.y_matrix + y_offset[i];
            neighbors[i] = neighbor;
            if (is_valid_loc_matrix(neighbor) && map_matrix[neighbor.x_matrix][neighbor.y_matrix] == 0)
                validity_neighbors[i] = true;
            else 
                validity_neighbors[i] = false;
        }

        for (int i = 0; i < 8; i++) { //for all neighbors
            loc_matrix neighbor = neighbors[i];
            if (i == 0)
                if (validity_neighbors[i] && (map_matrix[neighbors[1].x_matrix][neighbors[1].y_matrix] == 1 || map_matrix[neighbors[7].x_matrix][neighbors[7].y_matrix] == 1))
                    validity_neighbors[i] = false;
            if (i == 2)
                if (validity_neighbors[i] && (map_matrix[neighbors[3].x_matrix][neighbors[3].y_matrix] == 1 || map_matrix[neighbors[1].x_matrix][neighbors[1].y_matrix] == 1))
                    validity_neighbors[i] = false;
            if (i == 4)
                if (validity_neighbors[i] && (map_matrix[neighbors[5].x_matrix][neighbors[5].y_matrix] == 1 || map_matrix[neighbors[3].x_matrix][neighbors[3].y_matrix] == 1))
                    validity_neighbors[i] = false;
            if (i == 6)
                if (validity_neighbors[i] && (map_matrix[neighbors[7].x_matrix][neighbors[7].y_matrix] == 1 || map_matrix[neighbors[5].x_matrix][neighbors[5].y_matrix] == 1))
                    validity_neighbors[i] = false;
            
            if (validity_neighbors[i]) {
                double new_cost = cost_so_far[current_matrix.x_matrix][current_matrix.y_matrix] + neighbor_cost[i];
                if (cost_so_far[neighbor.x_matrix][neighbor.y_matrix] > 100000.0 || new_cost < cost_so_far[neighbor.x_matrix][neighbor.y_matrix]) {
                    cost_so_far[neighbor.x_matrix][neighbor.y_matrix] = new_cost;
                    double priority = new_cost + heuristic(neighbor, goal_matrix);
                    frontier.emplace(priority, neighbor);
                    came_from[neighbor.x_matrix][neighbor.y_matrix] = current_matrix;
                }
            }
            else {
                //std::cout << "[WARNING]: Neighbor " << i << " of (" << current_matrix.x_matrix << ", " << current_matrix.y_matrix << "). Accessed invalid area: (" << neighbor.x_matrix << ", " << neighbor.y_matrix << ")" << std::endl;
            }
        }
    }
}

std::deque<loc_real> print_path(loc_matrix& start_matrix, loc_matrix& goal_matrix, std::array<std::array<loc_matrix, 20>, 18>& came_from, std::array<std::array<double, 20>, 18>& cost_so_far) {
    std::deque<loc_real> shortest_path, temp;
    loc_matrix current_matrix = goal_matrix;
    if (cost_so_far[current_matrix.x_matrix][current_matrix.y_matrix] < 100000) { // if (was the goal even reached?)
        std::cout << std::endl << "(" << current_matrix.x_matrix << ", " << current_matrix.y_matrix << ") -> ";
        shortest_path.push_front(matrix_to_real(current_matrix));
        while (!(current_matrix == start_matrix)) {
            current_matrix = came_from[current_matrix.x_matrix][current_matrix.y_matrix];
            shortest_path.push_front(matrix_to_real(current_matrix));
        }
        std::cout << std::endl;

        temp = shortest_path;

        while (!temp.empty()) {
            ROS_INFO("(%f, %f) -> ", temp.front().x_real, temp.front().y_real);
            //std::cout << "(" << temp.front().x_real << ", " << temp.front().y_real << ") -> ";
            temp.pop_front();
        }

    }
    else
    {
        ROS_INFO("Goal wasn't reached!");
        //std::cout << "Goal wasn't reached!" << std::endl;
    }

    return shortest_path;
}



//ROS specific initializations
ros::Publisher cmd_pub;

inline double dist(double x1, double y1, double x2, double y2) {
    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}


void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg){   

    double current_pose_x, current_pose_y, current_pose_theta;

    // linear position
    current_pose_x = odom_msg->pose.pose.position.x;
    current_pose_y = odom_msg->pose.pose.position.y;
    
    // quaternion to RPY conversion
    tf::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // angular position
    current_pose_theta = yaw;

    ros::NodeHandle n;
    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    geometry_msgs::Twist motion_msg;

    double target_pose_x = shortest_path.front().x_real, target_pose_y = shortest_path.front().y_real;
    //double target_pose_x = -3.5, target_pose_y = -6.0; // shortest_path.front().x_real;
    double distance_to_target = dist(current_pose_x, current_pose_y, target_pose_x, target_pose_y);
    double angle_to_target = atan((target_pose_y - current_pose_y)/(target_pose_x - current_pose_x)) - current_pose_theta;
    if(std::abs(angle_to_target) > 0.05){
        motion_msg.linear.x = 0;
        motion_msg.angular.z = 0.8;
    }
    else{
        if(distance_to_target > 0.05){ //try diff values
            
            motion_msg.linear.x = 0.7; //0.2
            motion_msg.angular.z = 0;
            distance_to_target = 0.1;
        }
        else{
        motion_msg.linear.x = 0;
        motion_msg.angular.z = 0;
        if(!shortest_path.empty())
            shortest_path.pop_front();
        }
    }
    cmd_pub.publish(motion_msg);

    //if (distance_to_target < 0.2) shortest_path.pop_front();
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar1");
    ros::NodeHandle n;

    loc_real start_real, goal_real;
    n.getParam("goalx", goal_real.x_real);
    n.getParam("goaly", goal_real.y_real);
    start_real.x_real = -8.0;
    start_real.y_real = -2.01;
    
    
    //goal_real.x_real = 4.5;
    //goal_real.y_real = 9.0;
    /*
    start_real.x_real = -8.0;
    start_real.y_real = -2.01;
    goal_real.x_real = 4.5;
    goal_real.y_real = 9.0;
    */

    std::array<std::array<int, 18>, 20> map_matrix_compl = { {
                                                      {0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
                                                      {0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
                                                      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                      {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                      {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                      {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
                                                      {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
                                                      {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0},
                                                      {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
                                                      {0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
                                                      {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1},
                                                      {0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0},
                                                      {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
                                                      {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                                                      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                                      {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                                                      {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0},
                                                      {0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0},
                                                      {0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0},
                                                      {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1}
                                                    } };

    std::array<std::array<int, 20>, 18> map_matrix;

    for (int i = 0; i < 18; i++)
        for (int j = 0; j < 20; j++)
            map_matrix[i][j] = map_matrix_compl[j][i];

    

    loc_matrix start_matrix, goal_matrix, default_matrix; 
    start_matrix = real_to_matrix(start_real);
    goal_matrix = real_to_matrix(goal_real);
    default_matrix.x_matrix = -1;
    default_matrix.y_matrix = -1;
 
    std::array<std::array<loc_matrix, 20>, 18> came_from;
    for (int i = 0; i < 18; i++)
        for (int j = 0; j < 20; j++)
            came_from[i][j] = default_matrix;

    std::array<std::array<double, 20>, 18> cost_so_far;
    for (int i = 0; i < 18; i++)
        for (int j = 0; j < 20; j++)
            cost_so_far[i][j] = (double)INT_MAX;

    a_star_search(map_matrix, start_matrix, goal_matrix, came_from, cost_so_far); 

    shortest_path = print_path(start_matrix, goal_matrix, came_from, cost_so_far);
    bkp_shortest_path = shortest_path;
    shortest_path.pop_front();


    ROS_INFO("Hi\n");
    ros::Rate loop_rate(10);
    
    ros::Subscriber odom_sub = n.subscribe("base_pose_ground_truth", 1000, odom_callback);

    ros::spin();

    return 0;
}


