/**
 * Central node for the simulation
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/10/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPCentralNode.hpp"

MaxMinLPCentralNode::MaxMinLPCentralNode() {
	m_nh.param("/max_min_lp_simulation_central_node/num_robot", m_num_robot, 1);

	// Services
	m_service = m_nh.advertiseService("/robot_request", &MaxMinLPCentralNode::initialize, this);

	m_request_robot_id = 1;
	m_send_robot_id = 1;

	// Subscribers
	// ros::Subscriber robot_1_request_sub = nh.subscribe("/max_min_lp_robot_node/robot_1/request", 1000, countRequestCallback);
	// ros::Subscriber robot_2_request_sub = nh.subscribe("/max_min_lp_robot_node/robot_2/request", 1000, countRequestCallback);
}

bool MaxMinLPCentralNode::initialize(max_min_lp_simulation::MessageRequest::Request &req, max_min_lp_simulation::MessageRequest::Response &res) {
	if (strcmp(req.state_check.c_str(), "ready") == 0) {
		if (m_request_robot_id == req.robot_id) {
			m_robot_info.robot_status.push_back(string("initialized"));
			// m_robot_info.robot_neighbor.push_back
			res.state_answer = "wait";
			m_request_robot_id += 1;
			m_send_robot_id = 1;
		}

		if (m_request_robot_id == (m_num_robot+1)) {
			if (m_send_robot_id == req.robot_id) {
				res.state_answer = "start";
				m_send_robot_id += 1;

				if (m_send_robot_id == (m_num_robot+1)) {
					m_request_robot_id = 1;
				}
			}
		}
	}

	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_central_node");

	MaxMinLPCentralNode cn;

	ros::spin();

	return 0;
}