/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * Important note: in this code a robot is used to take into consideration of the reduction of the step 1 of local algorithm. 
 * Thus, a robot here does not indicate a real robot but an element of a real robot. Instead, ROBOT is used for a real robot.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/27/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/get_odom.hpp"

get_odom::get_odom() : 
m_num_robot(1), m_num_target(1), m_robot_id(1), m_robot_name(string("robot_0")), m_private_nh("~") 
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("robot_id", m_robot_id);
	m_private_nh.getParam("robot_name", m_robot_name);

	m_odom_sub = m_nh.subscribe("/gazebo/model_states", 1000, &get_odom::updateOdom, this);
	m_odom_service = m_nh.advertiseService("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/odom_request", &get_odom::return_odom, this);
}

void get_odom::updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg) {
	int size_msg = m_num_robot + m_num_target + 1; // 1 is for 'ground plane' in gazebo.
	int id = -1;
	for (int i = 0; i < size_msg; i++) {
		if (strcmp(msg->name[i].c_str(), m_robot_name.c_str()) == 0) {
			id = i;
		}
	}
	if (id != -1) {
		m_pos = msg->pose[id];
	}
}

bool get_odom::return_odom(max_min_lp_simulation::GetOdom::Request &req, max_min_lp_simulation::GetOdom::Response &res) {
	res.return_odom = m_pos;
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "get_odom");

	get_odom gtom;

	ros::spin();
	return 0;
}