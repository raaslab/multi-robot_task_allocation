/**
 * Each robot node for the greedy algorithm.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \07/17/2018
 * Copyright 2018. All Rights Reserved.
 */

#include "max_min_lp_simulation/RobotGreedyJournal.hpp"

#define PHI 3.141592

RobotGreedyJournal::RobotGreedyJournal() :
m_num_robot(1), m_num_target(1), m_robot_id(1), m_robot_name(string("robot_1")), m_num_layer(2), 
m_verbal_flag(false), m_epsilon(0.1), m_num_motion_primitive(10), m_time_interval(10), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("robot_id", m_robot_id);
	m_private_nh.getParam("robot_name", m_robot_name);
	m_private_nh.getParam("num_layer", m_num_layer);
	m_private_nh.getParam("verbal_flag", m_verbal_flag);
	m_private_nh.getParam("epsilon", m_epsilon);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);
	m_private_nh.getParam("time_interval", m_time_interval);

	// Publishers
	// w_j

	// Subscribers
	// w_j
}

void RobotGreedyJournal::updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg) {
	int size_msg = m_num_robot + m_num_target + 1; // 1 is for 'ground plane' in gazebo.
	int id;
	for (int i = 0; i < size_msg; i++) {
		if (strcmp(msg->name[i].c_str(), m_robot_name.c_str()) == 0) {
			id = i;
		}
	}
	m_pos = msg->pose[id];
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_greedy_journal");

	RobotGreedyJournal rg;

	ros::spin();
	return 0;
}