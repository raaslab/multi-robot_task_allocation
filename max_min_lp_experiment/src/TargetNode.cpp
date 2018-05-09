/**
 * Target node for the experiment.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \05/08/2018
 * Copyright 2018. All Rights Reserved.
 */

#include "max_min_lp_experiment/TargetNode.hpp"

#define PI 3.141592

TargetNode::TargetNode() :
m_time_period(5), m_private_nh("~")
{
	m_private_nh.getParam("time_period", m_time_period);

	// Services
	m_request_service = m_nh.advertiseService("/target_request", &TargetNode::applyTargetMotion, this);

	targetInitialize();
	m_time_step = 0;
}

bool TargetNode::applyTargetMotion(max_min_lp_experiment::TargetRequest::Request &req, max_min_lp_experiment::TargetRequest::Response &res) {
	if (strcmp(req.state_request.c_str(), "center_ready") == 0) {
		m_time_step += 1;

		for (int i = 0; i < m_num_target; i++) {
			m_target_pose_x[i] += (m_target_velocity[i]*cos(m_target_orientation[i]*PI/180));
			m_target_pose_y[i] += (m_target_velocity[i]*sin(m_target_orientation[i]*PI/180));

			res.target_id.push_back(m_target_id[i]);
			geometry_msgs::Point each_target_pose;
			each_target_pose.x = m_target_pose_x[i];
			each_target_pose.y = m_target_pose_y[i];
			each_target_pose.z = 0;
			res.target_pose.push_back(each_target_pose);
		}

		res.state_answer = "target_done";
	}
}

void TargetNode::targetInitialize() {
	if (ros::param::get("target_node/num_of_targets", m_num_target)) {}
	else {
		ROS_WARN("Didn't find num_of_targets");
	}
	if (ros::param::get("target_node/target_id", m_target_id)) {}
	else {
		ROS_WARN("Didn't find target_id");
	}
	if (ros::param::get("target_node/pose_x", m_target_pose_x)) {}
	else {
		ROS_WARN("Didn't find pose_x");
	}
	if (ros::param::get("target_node/pose_y", m_target_pose_y)) {}
	else {
		ROS_WARN("Didn't find pose_y");
	}
	if (ros::param::get("target_node/velocity", m_target_velocity)) {}
	else {
		ROS_WARN("Didn't find velocity");
	}
	if (ros::param::get("target_node/orientation", m_target_orientation)) {}
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
