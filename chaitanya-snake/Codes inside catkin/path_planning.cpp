/* Author: Chaitanya Pb
   
   This C++ file integrates with ROS and creates a controller which makes the simulated snake robot follow the path 
   specified in the vg_path file.

   Uses these files to run: vg_path, sensor_data.txt, turn_data.txt

   Should be run only while the Gazebo simulation is running in another terminal.

   Contains paths to external files. */

#include <math.h>
#include <iostream>
#include <fstream>
#include <bits/stdc++.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/LaserScan.h"

#define DIST 0.6
#define PI 3.14159265
#define DELTA (PI/4)
#define PI_IN_SIM 0.6
#define TURN_TIME_MAX 250

double current_heading_wrt_x = PI;
double phinot = 0.0;
double omega = 0.02;
double alpha = 0.5;
int checkpoint_cnt = 0;

// Define sonar data array
std::vector<float> sonar_arr(16, 10.0);

// Define struct for coordinates
struct coordinates
{
	float x, y;
} current, target;

// Function computes Euclidean distance
double computeEdist(coordinates* ptra, coordinates* ptrb)
{
	double edist = sqrt(pow(ptra->x - ptrb->x, 2.0) + pow(ptra->y - ptrb->y, 2.0));
	return edist;
}

// Function implements serpenoid curve
double computeHirose(int t, int jnum, float alpha, float omega, float phinot)
{
	double phi = alpha*sin(-omega*t + (jnum-1)*DELTA) + phinot;
	return phi;
}

// Function computes the turn needed to reach next checkpoint
double computeHeading(coordinates* current, coordinates* target)
{
	double target_heading_wrt_x;
	if(target->x != 0 || target->y != 0)
	{
		target_heading_wrt_x = atan2(target->y - current->y, target->x - current->x);
		if(target_heading_wrt_x < 0)
		{
			target_heading_wrt_x = 2*PI + target_heading_wrt_x;
		}
	}
	else
	{
		target_heading_wrt_x = current_heading_wrt_x;
	}
	double req_turn = current_heading_wrt_x - target_heading_wrt_x;
	current_heading_wrt_x = target_heading_wrt_x;
	return req_turn*(PI_IN_SIM/PI);
}

// Function is called upon receiving state information of the snake robot
void posecallback(const gazebo_msgs::ModelStates::ConstPtr& state)
{
	current.x = state->pose.back().position.x;
	current.y = state->pose.back().position.y;
	return;
}

// Function is called upon receiving sonar data from the snake robot
void sonarcallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	float average = std::accumulate(scan->ranges.begin(), scan->ranges.end(), 0.0)/scan->ranges.size();
	int arr_loc;
	if(scan->header.frame_id == "sonar_left")
	{
		arr_loc = 0;
	}
	else if(scan->header.frame_id == "sonar_right")
	{
		arr_loc = 1;
	}
	else if(scan->header.frame_id == "2sonar_left")
	{
		arr_loc = 2;
	}
	else if(scan->header.frame_id == "2sonar_right")
	{
		arr_loc = 3;
	}
	else if(scan->header.frame_id == "3sonar_left")
	{
		arr_loc = 4;
	}
	else if(scan->header.frame_id == "3sonar_right")
	{
		arr_loc = 5;
	}
	else if(scan->header.frame_id == "4sonar_left")
	{
		arr_loc = 6;
	}
	else if(scan->header.frame_id == "4sonar_right")
	{
		arr_loc = 7;
	}
	else if(scan->header.frame_id == "5sonar_left")
	{
		arr_loc = 8;
	}
	else if(scan->header.frame_id == "5sonar_right")
	{
		arr_loc = 9;
	}
	else if(scan->header.frame_id == "6sonar_left")
	{
		arr_loc = 10;
	}
	else if(scan->header.frame_id == "6sonar_right")
	{
		arr_loc = 11;
	}
	else if(scan->header.frame_id == "7sonar_left")
	{
		arr_loc = 12;
	}
	else if(scan->header.frame_id == "7sonar_right")
	{
		arr_loc = 13;
	}
	else if(scan->header.frame_id == "8sonar_left")
	{
		arr_loc = 14;
	}
	else if(scan->header.frame_id == "8sonar_right")
	{
		arr_loc = 15;
	}
	sonar_arr[arr_loc] = average;
	return;
}

// Function prints important variables
void printStatus()
{
	std::cout << "Checkpoint Count = " << checkpoint_cnt << std::endl;
	std::cout << "New target = (" << target.x << ", " << target.y << ")" << std::endl;
	std::cout << "Current pose = (" << current.x << ", " << current.y << ")" << std::endl;
	std::cout << "Current heading = " << current_heading_wrt_x << std::endl;
	std::cout << "Req. Turn = " << phinot << std::endl;
	std::cout << "---------------------------" << std::endl;
	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planning");
	ros::NodeHandle nodeh;

	// Define topic publishers and subscribers
	ros::Publisher jointb2 = nodeh.advertise<std_msgs::Float64>("/snake_robot/b_2_position_controller/command",1);
	ros::Publisher joint23 = nodeh.advertise<std_msgs::Float64>("/snake_robot/2_3_position_controller/command",1);
	ros::Publisher joint34 = nodeh.advertise<std_msgs::Float64>("/snake_robot/3_4_position_controller/command",1);
	ros::Publisher joint45 = nodeh.advertise<std_msgs::Float64>("/snake_robot/4_5_position_controller/command",1);
	ros::Publisher joint56 = nodeh.advertise<std_msgs::Float64>("/snake_robot/5_6_position_controller/command",1);
	ros::Publisher joint67 = nodeh.advertise<std_msgs::Float64>("/snake_robot/6_7_position_controller/command",1);
	ros::Publisher joint78 = nodeh.advertise<std_msgs::Float64>("/snake_robot/7_8_position_controller/command",1);
	ros::Subscriber robot_pose = nodeh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, posecallback);
	ros::Subscriber sl1 = nodeh.subscribe<sensor_msgs::LaserScan>("/sonar_left/scan", 1, sonarcallback);
	ros::Subscriber sr1 = nodeh.subscribe<sensor_msgs::LaserScan>("/sonar_right/scan", 1, sonarcallback);
	ros::Subscriber sl2 = nodeh.subscribe<sensor_msgs::LaserScan>("/2sonar_left/scan", 1, sonarcallback);
	ros::Subscriber sr2 = nodeh.subscribe<sensor_msgs::LaserScan>("/2sonar_right/scan", 1, sonarcallback);
	ros::Subscriber sl3 = nodeh.subscribe<sensor_msgs::LaserScan>("/3sonar_left/scan", 1, sonarcallback);
	ros::Subscriber sr3 = nodeh.subscribe<sensor_msgs::LaserScan>("/3sonar_right/scan", 1, sonarcallback);
	ros::Subscriber sl4 = nodeh.subscribe<sensor_msgs::LaserScan>("/4sonar_left/scan", 1, sonarcallback);
	ros::Subscriber sr4 = nodeh.subscribe<sensor_msgs::LaserScan>("/4sonar_right/scan", 1, sonarcallback);
	ros::Subscriber sl5 = nodeh.subscribe<sensor_msgs::LaserScan>("/5sonar_left/scan", 1, sonarcallback);
	ros::Subscriber sr5 = nodeh.subscribe<sensor_msgs::LaserScan>("/5sonar_right/scan", 1, sonarcallback);
	ros::Subscriber sl6 = nodeh.subscribe<sensor_msgs::LaserScan>("/6sonar_left/scan", 1, sonarcallback);
	ros::Subscriber sr6 = nodeh.subscribe<sensor_msgs::LaserScan>("/6sonar_right/scan", 1, sonarcallback);
	ros::Subscriber sl7 = nodeh.subscribe<sensor_msgs::LaserScan>("/7sonar_left/scan", 1, sonarcallback);
	ros::Subscriber sr7 = nodeh.subscribe<sensor_msgs::LaserScan>("/7sonar_right/scan", 1, sonarcallback);
	ros::Subscriber sl8 = nodeh.subscribe<sensor_msgs::LaserScan>("/8sonar_left/scan", 1, sonarcallback);
	ros::Subscriber sr8 = nodeh.subscribe<sensor_msgs::LaserScan>("/8sonar_right/scan", 1, sonarcallback);
	ros::Rate loop_rate(100);

	// Open needed files
	std::ifstream infile("/home/chaitanya_pb/Desktop/Work/BTP/Path_Planning/vg_path");
	std::ofstream outfile_data("/home/chaitanya_pb/Desktop/Work/BTP/Path_Planning/sensor_data.txt", std::fstream::out | std::fstream::app);
	std::ofstream outfile_turn("/home/chaitanya_pb/Desktop/Work/BTP/Path_Planning/turn_data.txt", std::fstream::out | std::fstream::app);
	outfile_data << "--------------------" << std::endl;
	outfile_turn << "----------" << std::endl; 

	int time = 0;

	std_msgs::Float64 floatmsg_b2, floatmsg_23, floatmsg_34;
	std_msgs::Float64 floatmsg_45, floatmsg_56, floatmsg_67, floatmsg_78;

	// Reach next checkpoint
	while(infile >> target.x >> target.y)
	{
		target.x = -target.x;
		target.y = -target.y;

		// Compute required turn
		phinot = computeHeading(&current, &target);
		int turn_time = 0;
		int checkpoint_time = 0;
		checkpoint_cnt++;

		printStatus();

		// Go towards target checkpoint
		while(ros::ok() && computeEdist(&current, &target) > DIST)
		{
			// Compute serpenoid curve value
			floatmsg_b2.data = computeHirose(time, 1, alpha, omega, phinot);
			floatmsg_23.data = computeHirose(time, 2, alpha, omega, phinot);
			floatmsg_34.data = computeHirose(time, 3, alpha, omega, phinot);
			floatmsg_45.data = computeHirose(time, 4, alpha, omega, phinot);
			floatmsg_56.data = computeHirose(time, 5, alpha, omega, phinot);
			floatmsg_67.data = computeHirose(time, 6, alpha, omega, phinot);
			floatmsg_78.data = computeHirose(time, 7, alpha, omega, phinot);

			// Publish commands via publishers
			jointb2.publish(floatmsg_b2);
			joint23.publish(floatmsg_23);
			joint34.publish(floatmsg_34);
			joint45.publish(floatmsg_45);
			joint56.publish(floatmsg_56);
			joint67.publish(floatmsg_67);
			joint78.publish(floatmsg_78);

			// Stop turn after enough time
			if(turn_time > TURN_TIME_MAX)
			{
				phinot = 0.0;
				turn_time = 0;
			}
			else
			{
				turn_time++;
			}

			// Save sonar and turn data to files regularly
			if(time%100 == 0)
			{
				for(int i = 0; i < sonar_arr.size(); i++)
		  		{
		  			outfile_data << sonar_arr[i] << "\t";
		  		}
		  		outfile_data << std::endl;
		  		outfile_turn << phinot << std::endl;
		  		std::cout << "Data saved " << sonar_arr.size() << std::endl;
			}

			// Check if target checkpoint missed
			if(checkpoint_time > 2500)
			{
				return 0;
			}
			else
			{
				checkpoint_time++;
			}

			ros::spinOnce();
			loop_rate.sleep();
			time = (time == 62831)? 0 : time+1;
		}

		// Save sonar and turn data to files
  		for(int i = 0; i < sonar_arr.size(); i++)
  		{
  			outfile_data << sonar_arr[i] << "\t";
  		}
  		outfile_data << std::endl;
  		outfile_turn << phinot << std::endl;
  		std::cout << "Data saved " << sonar_arr.size() << std::endl;
	}

	infile.close();
	outfile_data.close();
	outfile_turn.close();
	return 0;
}