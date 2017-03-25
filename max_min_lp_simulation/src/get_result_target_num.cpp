/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * Important note: in this code a robot is used to take into consideration of the reduction of the step 1 of local algorithm. 
 * Thus, a robot here does not indicate a real robot but an element of a real robot. Instead, ROBOT is used for a real robot.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/27/2017
 * Copyright 2017. All Rights Reserved.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/lexical_cast.hpp>
#include <max_min_lp_simulation/GetTotalNumTarget.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <ctime>

using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "get_total_num_target");
	ros::NodeHandle nh;

	ros::ServiceClient total_num_target_client;
	ofstream outputFile;

	outputFile.open("/home/yoon/yoon/max_min_ws/src/max_min_lp_simulation/data/greedy/results.txt");

	int return_total_num_target = 0;

	ros::Rate r(0.5); // 10 hz

	while (1)
	{
		clock_t current_time = clock();

		cout<<"Time = "<<current_time<<endl;

		total_num_target_client = nh.serviceClient<max_min_lp_simulation::GetTotalNumTarget>("/total_num_target_request");
		max_min_lp_simulation::GetTotalNumTarget srv;
		srv.request.request_odom = string("request");

		if (total_num_target_client.call(srv)) {
			return_total_num_target = srv.response.total_num_target;
			outputFile<<return_total_num_target<<endl;
		}
		ros::spinOnce();
		r.sleep();
	}

	outputFile.close();

	return 0;
}