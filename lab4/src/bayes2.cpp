#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/String.h>
#include <vector>
#include <lab4/Motion.h>
#include <lab4/Observation.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>
#include <array>
#include <iostream>
#include <math.h>
#include <time.h>
#include <algorithm>
#include <fstream>
#include <stdio.h>

#define foreach BOOST_FOREACH

const int x_count = 35, y_count = 35, theta_count = 16;
const double sd_xy = 0.1 , sd_theta = 0.2;

struct motion_msg
{
	int time_tag;
	double rot1_yaw;
	double trans;
	double rot2_yaw;
};

struct observation_msg
{
	int time_tag;
	int tag_num;
	double range;
	double bear_yaw;
};

struct grid_pos
{
    int x_pos;
    int y_pos;
    int theta_pos;
};

static const double inv_sqrt_s_2pi_xy = 0.0398942280401433;
static const double inv_sqrt_s_2pi_t = 0.0199471140200716;
double normal_pdf_xy(double x, double m) 
{    
    return inv_sqrt_s_2pi_xy * std::exp(-0.5f * (x-m) * (x-m) * 100);
}

double normal_pdf_t(double x, double m) 
{
    return inv_sqrt_s_2pi_t * std::exp(-0.5f * (x - m) * (x - m) * 25);
}


double yaw_from_quaternion(double x, double y, double z, double w) 
{
	double roll, pitch, yaw;
	tf::Quaternion q(x,y,z,w);
	tf::Matrix3x3 matrix1(q);
	matrix1.getRPY(roll, pitch, yaw);

	while(yaw < 0)
	 	yaw += (2 * M_PI);
	while(yaw>(2*M_PI))
        yaw -= 2*M_PI;

	return yaw;
}

std::array<std::array<std::array<double, theta_count>, y_count>, x_count> normalize_grid(std::array<std::array<std::array<double, theta_count>, y_count>, x_count> grid_t)
{
    auto grid1 = std::array<std::array<std::array<double, theta_count>, y_count>, x_count>();
    double sum = 0.0;
    for (int i = 0; i < x_count; ++i)
        for (int j = 0; j < y_count; ++j)
            for (int k = 0; k < theta_count; ++k)
                sum += grid_t[i][j][k];
    if (sum != 0)
        for (int i = 0; i < x_count; ++i)
            for (int j = 0; j < y_count; ++j)
                for (int k = 0; k < theta_count; ++k)
                    grid1[i][j][k] = grid_t[i][j][k] / sum;
    return grid1;
}

std::array<std::array<std::array<double, theta_count>, y_count>, x_count> initialize_grid(std::array<std::array<std::array<double, theta_count>, y_count>, x_count> grid_t, grid_pos initial_grid_pos)
{
    auto grid1 = std::array<std::array<std::array<double, theta_count>, y_count>, x_count>();
    grid_t[initial_grid_pos.x_pos][initial_grid_pos.y_pos][initial_grid_pos.theta_pos] = 1;
    double x_mean = ((double)initial_grid_pos.x_pos + 1) / 5 - 0.1, y_mean = ((double)initial_grid_pos.y_pos + 1) / 5 - 0.1, theta_mean = ((double)initial_grid_pos.theta_pos + 1) * 0.4 - 0.2;
    double gaussian_x, gaussian_y, gaussian_theta;
    for (int i = 0; i < x_count; ++i)
    {
        gaussian_x = normal_pdf_xy( (((double)i + 1)/5 - 0.1), x_mean);
        for (int j = 0; j < y_count; ++j) 
        {
            gaussian_y = normal_pdf_xy( (((double)j + 1)/5 - 0.1), y_mean);
            for (int k = 0; k < theta_count; ++k)
            {
                gaussian_theta = normal_pdf_t( (((double)k + 1)*0.4 - 0.2), theta_mean);
                grid1[i][j][k] = gaussian_x * gaussian_y * gaussian_theta * grid_t[initial_grid_pos.x_pos][initial_grid_pos.y_pos][initial_grid_pos.theta_pos];
            }
        }
    }
    return normalize_grid(grid1);
}


int main()
{
	//Initialize the vector of structs to store the rosbag data into
	std::vector<motion_msg> motion_msgs;
	std::vector<observation_msg> observation_msgs;

	//Read from bag file and store into appropriate vector of structs
	rosbag::Bag bag;
	bag.open("/home/first/catkin_ws/src/lab4/grid.bag", rosbag::bagmode::Read);
	std::vector<std::string> topics;
	topics.push_back(std::string("Movements"));
	topics.push_back(std::string("Observations"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	foreach(rosbag::MessageInstance const m, view)
	{
	    lab4::Motion::ConstPtr lm = m.instantiate<lab4::Motion>();
	    if (lm != NULL)
			motion_msgs.push_back({lm->timeTag, yaw_from_quaternion(lm->rotation1.x,lm->rotation1.y,lm->rotation1.z,lm->rotation1.w), lm->translation, yaw_from_quaternion(lm->rotation2.x,lm->rotation2.y,lm->rotation2.z,lm->rotation2.w)});

		lab4::Observation::ConstPtr lo = m.instantiate<lab4::Observation>();
	    if (lo != NULL)
			observation_msgs.push_back({lo->timeTag, lo->tagNum, lo->range, yaw_from_quaternion(lo->bearing.x, lo->bearing.y, lo->bearing.z, lo->bearing.w)});
	}
	bag.close();

	//create initial grid
	//initial position of robot is (12,28,9((200.52*M_PI/180)*2.5+1))
	// input:  grid1, grid_locations of robot 
	// output: grid1 with initial gaussian applied. don't return use void.
	grid_pos initial_grid_pos = {11,27,8}; //because array starts with 0. Output and calculation is with 1 as starting.
	std::array<std::array<double, 2>, 6> tag_locs = { { {1.25, 5.25},
                                                    {1.25, 3.25},
                                                    {1.25, 1.25},
                                                    {4.25, 1.25},
                                                    {4.25, 3.25},
                                                    {4.25, 5.25} } };
	auto grid1 = std::array<std::array<std::array<double, theta_count>, y_count>, x_count>();
	grid1 = initialize_grid(grid1, initial_grid_pos);

	clock_t t = clock();
	double p1=0, p2=0, p3=0, summation=0, del_rot1=0, del_trans=0, del_rot2=0, rep_rot1=0, rep_trans=0, rep_rot2=0, xn_c=0, xo_c=0, yn_c=0, yo_c=0, tn_c=0, to_c=0;
	double p4=0, p5=0, calc_range=0, calc_bear=0;
	
	std::array<int, 89> timestep;
	std::array<int, 89> x_store;
	std::array<int, 89> y_store;
	std::array<int, 89> t_store;
	
	//in a loop to process all messages, perform motion and observation
	for(int msg_count = 0; msg_count < std::min(motion_msgs.size(), observation_msgs.size()); ++msg_count)
	//for(int msg_count = 0; msg_count <  2; ++msg_count) //debug only
	{
		//motion model
		auto bel_grid_motion = std::array<std::array<std::array<double, theta_count>, y_count>, x_count>();
		rep_rot1 = motion_msgs[msg_count].rot1_yaw;
		rep_trans = motion_msgs[msg_count].trans;
		rep_rot2 = motion_msgs[msg_count].rot2_yaw;
		for (int x_n = 0; x_n < x_count; ++x_n)
		{
			xn_c = ((double)x_n + 1)/5 - 0.1;
        	for (int y_n = 0; y_n < y_count; ++y_n)
			{
				yn_c = ((double)y_n + 1)/5 - 0.1;
            	for (int t_n = 0; t_n < theta_count; ++t_n)
				{
					tn_c = ((double)t_n + 1)*0.4 - 0.2;
					summation = 0.0;
					for (int x_o = 0; x_o < x_count; ++x_o)
					{
						xo_c = ((double)x_o + 1)/5 - 0.1;
						for (int y_o = 0; y_o < y_count; ++y_o)
						{
							yo_c = ((double)y_o + 1)/5 - 0.1;
							del_trans = sqrt(pow((xn_c - xo_c), 2.0) + pow((yn_c - yo_c), 2.0));
							p2 = normal_pdf_xy(del_trans, rep_trans);
							for (int t_o = 0; t_o < theta_count; ++t_o)
							{
								to_c = ((double)t_o + 1)*0.4 - 0.2;
								del_rot1 = atan((yn_c - yo_c) / (xn_c - xo_c + 0.00001) ) - to_c;
                                while(del_rot1>(2*M_PI))
                                    del_rot1 -= 2*M_PI;
                                while(del_rot1<0)
                                    del_rot1 += 2*M_PI;
								del_rot2 = tn_c - to_c - del_rot1;
                                while(del_rot2>(2*M_PI))
                                    del_rot2 -= 2*M_PI;
                                while(del_rot2<0)
                                    del_rot2 += 2*M_PI;
								p1 = normal_pdf_t(del_rot1, rep_rot1);
								p3 = normal_pdf_t(del_rot2, rep_rot2);
								summation += (p1*p2*p3*grid1[x_o][y_o][t_o]); // Sigma(P(x_t|u_t,x_t-1)*Bel(x_t-1))
							}
						}
					}
					bel_grid_motion[x_n][y_n][t_n] = summation;
				}
			}
		}

		bel_grid_motion = normalize_grid(bel_grid_motion);

		//observation model
		double tag_x = tag_locs[observation_msgs[msg_count].tag_num][0]; //x
		double tag_y = tag_locs[observation_msgs[msg_count].tag_num][1]; //y
		double tag_range = observation_msgs[msg_count].range;
		double tag_bear = observation_msgs[msg_count].bear_yaw;
		auto bel_grid_observe = std::array<std::array<std::array<double, theta_count>, y_count>, x_count>();
		for (int x_n = 0; x_n < x_count; ++x_n)
		{
			xn_c = ((double)x_n + 1)/5 - 0.1 - tag_x;
        	for (int y_n = 0; y_n < y_count; ++y_n)
			{
				yn_c = ((double)y_n + 1)/5 - 0.1 - tag_y;
				p4 = normal_pdf_xy((sqrt(pow(xn_c, 2.0) + pow(yn_c, 2.0))), tag_range);
            	for (int t_n = 0; t_n < theta_count; ++t_n)
				{
                    calc_bear = (atan(yn_c / xn_c) - (((double)t_n + 1)*0.4 - 0.2));
                    while(calc_bear>(2*M_PI))
                        calc_bear -= 2*M_PI;
                    while(calc_bear<0)
                        calc_bear += 2*M_PI;
					p5 = normal_pdf_t(calc_bear, tag_bear);
					bel_grid_observe[x_n][y_n][t_n] = p4*p5*bel_grid_motion[x_n][y_n][t_n];
				}
			}
		}
		bel_grid_observe = normalize_grid(bel_grid_observe);


		t = clock() - t;
		std::cout << "At timeTag:"<< (msg_count+1)*2 <<" ,the robot is around :\n x: ";
		double max_lik_rob_loc = 0.0;
		int max_x = 0, max_y =  0, max_th = 0;
		for (int i = 0; i < x_count; ++i) 
		{
			for (int j = 0; j < y_count; ++j) 
			{
				for (int k = 0; k < theta_count; ++k) 
				{
					if(max_lik_rob_loc < bel_grid_observe[i][j][k])
					{
						max_lik_rob_loc = bel_grid_observe[i][j][k];
						max_x = i;
						max_y = j;
						max_th = k;
					}
				}
			}
		}

		timestep[msg_count] = (msg_count+1)*2;
		x_store[msg_count] = max_x;
		y_store[msg_count] = max_y;
		t_store[msg_count] = max_th;

	}

	FILE * pFile;
	pFile = fopen("/home/first/catkin_ws/src/lab4/trajectory.txt","w");
	for(int i=0;i<89;i++)
	{
		fprintf(pFile, "timeTag: %d, x: %d, y: %d, theta:%d\n",timestep[i],x_store[i],y_store[i],t_store[i]);
	}
	fclose(pFile);

	return 0;
}
