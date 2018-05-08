/**
 * Target node for the experiment.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \05/08/2018
 * Copyright 2018. All Rights Reserved.
 */

#include "max_min_lp_experiment/TargetNode.hpp"

TargetNode::TargetNode() :
m_time_period(5), m_private_nh("~")
{
	m_private_nh.getParam("time_period", m_time_period);

	// Services
	m_request_service = m_nh.advertiseService("/target_request", &TargetNode::applyTargetMotion, this);

	targetInitialize();
}

bool TargetNode::applyTargetMotion(max_min_lp_experiment::TargetRequest::Request &req, max_min_lp_experiment::TargetRequest::Response &res) {
}

void TargetNode::targetInitialize() {
	if (ros::param::get("central_node_greedy/num_of_targets", m_num_target)) {}
	else {
		ROS_WARN("Didn't find num_of_targets");
	}
	if (ros::param::get("central_node_greedy/target_id", m_target_id)) {}
	else {
		ROS_WARN("Didn't find target_id");
	}
	if (ros::param::get("central_node_greedy/pose_x", m_target_pose_x)) {}
	else {
		ROS_WARN("Didn't find pose_x");
	}
	if (ros::param::get("central_node_greedy/pose_y", m_target_pose_y)) {}
	else {
		ROS_WARN("Didn't find pose_y");
	}
	if (ros::param::get("central_node_greedy/velocity", m_target_velocity)) {}
	else {
		ROS_WARN("Didn't find velocity");
	}
	if (ros::param::get("central_node_greedy/orientation", m_target_orientation)) {}
	else {
		ROS_WARN("Didn't find orientation");
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "target_node");
	TargetNode tn;

	ros::spin();

	return 0;
}
