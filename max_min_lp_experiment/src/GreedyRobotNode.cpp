/**
 * Each robot node for the greedy algorithm in experiment.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \04/14/2018
 * Copyright 2018. All Rights Reserved.
 */

#include "max_min_lp_experiment/GreedyRobotNode.hpp"

GreedyRobotNode::GreedyRobotNode() :
m_robot_id(1), m_num_motion_primitive(10), m_time_period(5), m_robot_velocity(0.1), 
m_robot_angular_vel(0.1), m_private_nh("~")
{
	m_private_nh.getParam("robot_id", m_robot_id);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);
	m_private_nh.getParam("time_period", m_time_period);
	m_private_nh.getParam("robot_velocity", m_robot_velocity);
	m_private_nh.getParam("robot_angular_vel", m_robot_angular_vel);

	// Publishers
	m_after_motion_pub = m_nh.advertise<std_msgs::String>("/robot_after_motion", 1);
	m_cmd_vel_pub = m_nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
	// Subscribers
	m_response_sub = m_nh.subscribe("/center_response", 10, &GreedyRobotNode::applyMotionPrimitives, this);
	m_pose_sub = m_nh.subscribe("/RosAria/pose", 10, &GreedyRobotNode::getRobotPose, this);

	m_count_robotInitialize_activate = 0;
}

void GreedyRobotNode::applyMotionPrimitives(const std_msgs::String::ConstPtr& msg) {
	double cur_time = ros::Time::now().toSec();
	int time_for_orient = 2;
	int time_for_translate = 2;

	while (cur_time < time_for_orient) {
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0; cmd_vel.linear.y = 0; cmd_vel.linear.z = 0;
		cmd_vel.angular.x = 0; cmd_vel.angular.y = 0; cmd_vel.linear.z = m_robot_angular_vel;
		m_cmd_vel_pub.publish(cmd_vel);
	}

	// bool result_success = robotInitialize(); // Initialization through the central node.

	// if (result_success) {
	// 	std_msgs::String msg;
	//     stringstream ss;
	//     ss<<"action is applied";
	//     msg.data = ss.str();
	//     m_after_motion_pub.publish(msg);
	// }
}

bool GreedyRobotNode::robotInitialize() {
	m_request_client = m_nh.serviceClient<max_min_lp_experiment::RobotRequest>("/robot_request");
	max_min_lp_experiment::RobotRequest srv;
	srv.request.robot_id = m_robot_id;
	srv.request.robot_pose = m_robot_pose;
	srv.request.state_request = "ready";

	// Compute motion primivites.
	if (m_count_robotInitialize_activate == 0) {
	  	m_motion_primitive_pose.clear();
		m_motion_primitive_pose = computeMotionPrimitives();
	}

	if (m_request_client.call(srv)) {
		if (strcmp(srv.response.state_answer.c_str(), "start") == 0) {
			// Obtain local information from the central node.
			srv.response.target_id;
			srv.response.target_pose;
			srv.response.comm_robot_id;

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

vector<geometry_msgs::Point> GreedyRobotNode::computeMotionPrimitives() {
	vector<geometry_msgs::Point> motion_primitive;
	motion_primitive.push_back(m_robot_pose); // First motion primitive is to stay in the same position.

	// if ((m_num_motion_primitive-1) % 2 == 0) { // Even number.
	// 	(m_num_motion_primitive-1)/2
	// }
	// else { // Odd number.

	// }

	return motion_primitive;
}

void GreedyRobotNode::getRobotPose(const nav_msgs::Odometry::ConstPtr& msg) {
	m_robot_pose.x = msg->pose.pose.position.x;
	m_robot_pose.y = msg->pose.pose.position.y;
	m_robot_pose.z = msg->pose.pose.position.z;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_node_greedy");
	GreedyRobotNode rn;

	ros::spin();
	return 0;
}
