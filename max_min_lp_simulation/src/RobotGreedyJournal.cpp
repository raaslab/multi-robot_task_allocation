/**
 * Each robot node for the greedy algorithm.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \07/17/2018
 * Copyright 2018. All Rights Reserved.
 */

#include "max_min_lp_simulation/RobotGreedyJournal.hpp"

#define PHI 3.141592

RobotGreedyJournal::RobotGreedyJournal() :
m_num_robot(1), m_num_target(1), m_robot_id(1), m_robot_name(string("robot_1")),  
m_verbal_flag(false), m_num_motion_primitive(10), m_time_interval(10), m_robot_time(1), 
m_sensing_range(5), m_comm_range(5), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("robot_id", m_robot_id);
	m_private_nh.getParam("robot_name", m_robot_name);
	m_private_nh.getParam("verbal_flag", m_verbal_flag);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);
	m_private_nh.getParam("time_interval", m_time_interval);
	m_private_nh.getParam("robot_time", m_robot_time);
	m_private_nh.getParam("sensing_range", m_sensing_range);
	m_private_nh.getParam("comm_range", m_comm_range);

	// Subscribers
	m_odom_sub = m_nh.subscribe("/gazebo/model_states", 1000, &RobotGreedyJournal::updateOdom, this);

	double cur_time =ros::Time::now().toSec();
	double prev_time =ros::Time::now().toSec();

	while (1) {
		// At every time interval, do the following.
		cur_time =ros::Time::now().toSec();
		if (cur_time-prev_time >= m_time_interval) {
			prev_time =ros::Time::now().toSec();
			
			// Initialization.
			m_cur_target_pos.clear();
			m_c.clear();
			m_w.clear();

			// Compute c (i.e., the tracking quality) by using the sensing range.
			m_target_odom_client = m_nh.serviceClient<max_min_lp_simulation::GetOdom>("/target_odom_request");
			max_min_lp_simulation::GetOdom srv;
			srv.request.request_odom = string("request");

			if (m_target_odom_client.call(srv)) {
				m_cur_target_pos = srv.response.return_target_odom;
			}
			else {
				ROS_ERROR("ERROR: Targets are not properly obtained.");
			}

			for (int i = 0; i < m_num_target; i++) {
				double dist_target_robot = sqrt((m_pos.position.x-m_cur_target_pos[i].position.x)*(m_pos.position.x-m_cur_target_pos[i].position.x)
					+(m_pos.position.y-m_cur_target_pos[i].position.y)*(m_pos.position.y-m_cur_target_pos[i].position.y));
				if (dist_target_robot >= m_sensing_range) {
					m_c.push_back(0);
				}
				else {
					m_c.push_back(1/dist_target_robot);
				}
			}

			// Compute the communication graph by using the communication range.
			
		}
	}

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