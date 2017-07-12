/* Author: Chaitanya Pb
   
   This C++ file integrates with ROS and creates a controller which performs lateral undulation with the simulated
   snake robot using Hirose's serpenoid curve.

   Standalone: It does not call functions from any other file.

   Should be run only while the Gazebo simulation is running in another terminal.

   Doesn't contain paths to external files. */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"

#define ALPHA 0.5
#define OMEGA 0.02
#define DELTA (3.1416/4)

// Function implements the serpenoid curve
double computeHirose(int t, int jnum, float phinot)
{
	double phi = ALPHA*sin(-OMEGA*t + (jnum-1)*DELTA) + phinot;
	return phi;
}

// Function is called upon receiving state information of the snake robot
void posecallback(const gazebo_msgs::ModelStates state)
{
	std::cout << state.pose.back().position;
	std::cout << "----------------" << std::endl;
	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lateral_undulation");
	ros::NodeHandle n;

	// Define topic publishers
	ros::Publisher lu_b2_pub = n.advertise<std_msgs::Float64>("/snake_robot/b_2_position_controller/command",1);
	ros::Publisher lu_23_pub = n.advertise<std_msgs::Float64>("/snake_robot/2_3_position_controller/command",1);
	ros::Publisher lu_34_pub = n.advertise<std_msgs::Float64>("/snake_robot/3_4_position_controller/command",1);
	ros::Publisher lu_45_pub = n.advertise<std_msgs::Float64>("/snake_robot/4_5_position_controller/command",1);
	ros::Publisher lu_56_pub = n.advertise<std_msgs::Float64>("/snake_robot/5_6_position_controller/command",1);
	ros::Publisher lu_67_pub = n.advertise<std_msgs::Float64>("/snake_robot/6_7_position_controller/command",1);
	ros::Publisher lu_78_pub = n.advertise<std_msgs::Float64>("/snake_robot/7_8_position_controller/command",1);

	ros::Subscriber robot_pose = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, posecallback);

	ros::Rate loop_rate(100);

	int time = 0;
	float phinot = 0;

	while(ros::ok())
	{
		std_msgs::Float64 sv_b2, sv_23, sv_34, sv_45, sv_56, sv_67, sv_78;

		// Compute serpenoid curve value
		sv_b2.data = computeHirose(time,1,phinot);
		sv_23.data = computeHirose(time,2,phinot);
		sv_34.data = computeHirose(time,3,phinot);
		sv_45.data = computeHirose(time,4,phinot);
		sv_56.data = computeHirose(time,5,phinot);
		sv_67.data = computeHirose(time,6,phinot);
		sv_78.data = computeHirose(time,7,phinot);

		// Publish commands via publishers
		lu_b2_pub.publish(sv_b2);
		lu_23_pub.publish(sv_23);
		lu_34_pub.publish(sv_34);
		lu_45_pub.publish(sv_45);
		lu_56_pub.publish(sv_56);
		lu_67_pub.publish(sv_67);
		lu_78_pub.publish(sv_78);

		ros::spinOnce();
		loop_rate.sleep();
		
		time = (time == 62831)? 0 : time+1;
	}
	return 0;
}