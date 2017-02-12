/**
 * Each robot node for the simulation
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/10/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPRobotNode.hpp"

MaxMinLPRobotNode::MaxMinLPRobotNode() :
m_robot_id(1), m_robot_name(string("robot_1")), m_num_layer(2), m_verbal_flag(false), m_epsilon(0.1), m_private_nh("~")
{
	m_private_nh.getParam("robot_id", m_robot_id);
	m_private_nh.getParam("robot_name", m_robot_name);
	m_private_nh.getParam("num_layer", m_num_layer);
	m_private_nh.getParam("verbal_flag", m_verbal_flag);
	m_private_nh.getParam("epsilon", m_epsilon);

	bool result_success = initialize();

	if (result_success) {
		ROS_INFO("The %s is initiated.", m_robot_name.c_str());
	}

	// Publishers
	// m_request_pub = m_nh.advertise<std_msgs::String>("/max_min_lp_robot_node/"+m_robot_name+"/request", 1);

	// Subscribers
	m_odom_sub = m_nh.subscribe("/max_min_lp_robot_node/"+m_robot_name+"/odom", 1000, &MaxMinLPRobotNode::updateOdom, this);
}

void MaxMinLPRobotNode::updateOdom(const nav_msgs::Odometry::ConstPtr& msg) {
	//Compute motion primitives
}

void MaxMinLPRobotNode::updateGraph() {
}

bool MaxMinLPRobotNode::initialize() {
	m_client = m_nh.serviceClient<max_min_lp_simulation::MessageRequest>("/robot_request");
	max_min_lp_simulation::MessageRequest srv;
	srv.request.robot_id = m_robot_id;
	srv.request.state_check = "ready";
	// My info
	if (m_client.call(srv)) {
		if (strcmp(srv.response.state_answer.c_str(), "start") == 0) {
			// m_local_info
			return true;
		}
		else {
			return MaxMinLPRobotNode::initialize();
		}
	}
	else {
		ROS_INFO("Fail to communicate with the server.");
		return false;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_robot_node");

	MaxMinLPRobotNode rn;

	ros::spin();
	return 0;
}