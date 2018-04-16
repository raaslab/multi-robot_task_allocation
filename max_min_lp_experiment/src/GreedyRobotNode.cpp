/**
 * Each robot node for the greedy algorithm in experiment.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \04/14/2018
 * Copyright 2018. All Rights Reserved.
 */

#include "max_min_lp_experiment/GreedyRobotNode.hpp"

GreedyRobotNode::GreedyRobotNode() :
m_robot_id(1), m_num_motion_primitive(10), m_private_nh("~")
{
	m_private_nh.getParam("robot_id", m_robot_id);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);

	// Publishers
	m_after_motion_pub = m_nh.advertise<std_msgs::String>("/robot_after_motion", 1);
	// Subscribers
	m_response_sub = m_nh.subscribe("/center_response", 10, &GreedyRobotNode::applyMotionPrimitives, this);

	m_count_robotInitialize_activate = 0;
}

void GreedyRobotNode::applyMotionPrimitives(const std_msgs::String::ConstPtr& msg) {
	bool result_success = robotInitialize();

	if (result_success) {
		ROS_INFO(" ");
		ROS_INFO("After applying motion primitives.");
		ROS_INFO(" ROBOT %d : (%.2f, %.2f)", m_robot_id, m_pos.position.x, m_pos.position.y);
		std_msgs::String msg;
	    stringstream ss;
	    ss<<"action is applied";
	    msg.data = ss.str();
	    m_after_motion_pub.publish(msg);
	}
}

bool GreedyRobotNode::robotInitialize() {
	m_request_client = m_nh.serviceClient<max_min_lp_experiment::RobotRequest>("/robot_request");
	max_min_lp_experiment::RobotRequest srv;
	srv.request.robot_id = m_robot_id;
	srv.request.state_request = "ready";

	// Compute motion primivites.
	if (m_count_robotInitialize_activate == 0) {
	  	m_motion_primitive_pose.clear();
		m_motion_primitive_pose = computeMotionPrimitives();
	}
	// srv.request.motion_primitive_info = m_motion_primitive_pose;

	if (m_request_client.call(srv)) {
		if (strcmp(srv.response.state_answer.c_str(), "start") == 0) {
			// m_local_info
			m_count_robotInitialize_activate = 0;
			return true;
		}
		else {
			m_count_robotInitialize_activate += 1;
			return GreedyRobotNode::robotInitialize();
		}
	}
	else {
		ROS_INFO("Fail to communicate with the server.");
		return false;
	}
}

vector<geometry_msgs::Pose> GreedyRobotNode::computeMotionPrimitives() {
	// At this moment, tweak this part. Just consider the case when the number of motion primitives considered is five. This should be changed later.
	// Also, this motion primitives are based on the 1m/s for x-direction and 1rad/s for z-direction.
	vector<geometry_msgs::Pose> temp_motion_primitive;

	return temp_motion_primitive;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_robot_node_greedy");
	GreedyRobotNode rn;

	ros::spin();
	return 0;
}