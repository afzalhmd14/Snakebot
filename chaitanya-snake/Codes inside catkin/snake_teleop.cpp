/* Author: Chaitanya Pb
   
   This C++ file integrates with ROS and creates a controller which performs lateral undulation by teleoperation
   with the simulated snake robot. That means the simulated snake robot can be controlled by the WASD keys.

   Standalone: It does not call functions from any other file.

   Should be run only while the Gazebo simulation is running in another terminal.

   Doesn't contain paths to external files. */

#include "ros/ros.h"
#include <termios.h>
#include "std_msgs/Float64.h"

#define DELTA (3.1416/4)

// Function implements the serpenoid curve
double computeHirose(int t, int jnum, float alpha, float omega, float phinot)
{
	double phi = alpha*sin(-omega*t + (jnum-1)*DELTA) + phinot;
	return phi;
}

// Function checks for key press
char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0)
		//ROS_INFO("no_key_pressed");
		;
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "snake_teleop");
	ros::NodeHandle n;

	// Define topic publishers
	ros::Publisher lu_b2_pub = n.advertise<std_msgs::Float64>("/snake_robot/b_2_position_controller/command",1);
	ros::Publisher lu_23_pub = n.advertise<std_msgs::Float64>("/snake_robot/2_3_position_controller/command",1);
	ros::Publisher lu_34_pub = n.advertise<std_msgs::Float64>("/snake_robot/3_4_position_controller/command",1);
	ros::Publisher lu_45_pub = n.advertise<std_msgs::Float64>("/snake_robot/4_5_position_controller/command",1);
	ros::Publisher lu_56_pub = n.advertise<std_msgs::Float64>("/snake_robot/5_6_position_controller/command",1);
	ros::Publisher lu_67_pub = n.advertise<std_msgs::Float64>("/snake_robot/6_7_position_controller/command",1);
	ros::Publisher lu_78_pub = n.advertise<std_msgs::Float64>("/snake_robot/7_8_position_controller/command",1);
	ros::Rate loop_rate(100);

	float phinot = 0;
	float omega = 0.02;
	float alpha = 0.5;

	int time = 0;
	char pressed;

	while(ros::ok())
	{
		std_msgs::Float64 sv_b2, sv_23, sv_34, sv_45, sv_56, sv_67, sv_78;

		// Compute serpenoid curve value
		sv_b2.data = computeHirose(time, 1, alpha, omega, phinot);
		sv_23.data = computeHirose(time, 2, alpha, omega, phinot);
		sv_34.data = computeHirose(time, 3, alpha, omega, phinot);
		sv_45.data = computeHirose(time, 4, alpha, omega, phinot);
		sv_56.data = computeHirose(time, 5, alpha, omega, phinot);
		sv_67.data = computeHirose(time, 6, alpha, omega, phinot);
		sv_78.data = computeHirose(time, 7, alpha, omega, phinot);

		pressed = getch();
		ROS_INFO("%f %f %f", alpha, omega, phinot);

		// Take action depending on key pressed
		switch(pressed)
		{
			case 'w':
				if(phinot != 0)
					phinot = 0;
				else
					omega += 0.02;
				break;
			case 'a':
				phinot = -0.25;
				break;
			case 's':
				if(phinot != 0)
					phinot = 0;
				else
					omega -= 0.02;
				break;
			case 'd':
				phinot = 0.25;
				break;
			default:
				break;
		}

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