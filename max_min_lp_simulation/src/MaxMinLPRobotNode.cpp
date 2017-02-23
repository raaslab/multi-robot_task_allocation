/**
 * Each robot node for the simulation
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/13/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPRobotNode.hpp"

#define PHI 3.141592

MaxMinLPRobotNode::MaxMinLPRobotNode() :
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
	m_general_node_pub = m_nh.advertise<max_min_lp_msgs::general_node_array>("/max_min_lp_msgs/general_node_array", 1);
	m_layered_node_pub = m_nh.advertise<max_min_lp_msgs::layered_node_array>("/max_min_lp_msgs/layered_node_array", 1);

	// Subscribers
	request_sub = m_nh.subscribe("/robot_status", 1000, &MaxMinLPRobotNode::applyMotionPrimitives, this);
	m_odom_sub = m_nh.subscribe("/gazebo/model_states", 1000, &MaxMinLPRobotNode::updateOdom, this);
}

void MaxMinLPRobotNode::updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg) {
	int size_msg = m_num_robot + m_num_target + 1; // 1 is for 'ground plane' in gazebo.
	int id;
	for (int i = 0; i < size_msg; i++) {
		if (strcmp(msg->name[i].c_str(), m_robot_name.c_str()) == 0) {
			id = i;
		}
	}
	m_pos = msg->pose[id];
}

void MaxMinLPRobotNode::applyMotionPrimitives(const std_msgs::String::ConstPtr& msg) {
	bool result_success = initialize();

	if (result_success) {
		// Publisher
		m_response_to_server_pub = m_nh.advertise<std_msgs::String>("/robot_comm_graph", 1);

		std_msgs::String msg;
	    stringstream ss;
	    ss<<"comm graph is complete";
	    msg.data = ss.str();
	    m_response_to_server_pub.publish(msg);

		if (m_verbal_flag) {
			ROS_INFO("The %s is initiated.", m_robot_name.c_str());
		}

		// Publisher for general nodes
		max_min_lp_msgs::general_node_array temp_msg;
		// Robot nodes
		for (int i = 0; i < m_gen_r_node.size(); i++) {
			temp_msg.gen_nodes.push_back(m_gen_r_node[i]);
		}
		// Motion primitive to robot nodes
		for (int i = 0; i < m_gen_p_r_node.size(); i++) {
			temp_msg.gen_nodes.push_back(m_gen_p_r_node[i]);
		}
		// Motion primitive to target nodes
		for (int i = 0; i < m_gen_p_t_node.size(); i++) {
			temp_msg.gen_nodes.push_back(m_gen_p_t_node[i]);
		}
		// Target nodes
		for (int i = 0; i < m_gen_t_node.size(); i++) {
			temp_msg.gen_nodes.push_back(m_gen_t_node[i]);
		}

		m_general_node_pub.publish(temp_msg);

		// Local algorithm is applied from here.
		max_min_lp_core::MaxMinLPDecentralizedCore lpc(m_robot_id, m_gen_r_node, m_gen_p_r_node, m_gen_p_t_node, m_gen_t_node,
			m_num_layer, m_verbal_flag, m_epsilon, m_max_neighbor_hop, m_num_neighbors_at_each_hop, m_ROBOT_num_robot, m_prev_accumulate_robot,
			m_num_survived_robot, m_ROBOT_num_motion_primitive, m_prev_accumulate_motion_primitive, m_num_survived_motion_primitive, m_constraint_value);

		// Step 2
		lpc.convertDecentralizedLayeredMaxMinLP();

		// Step 3
		//   Phase 1 and 2
		lpc.applyLocalAlgorithmPhase1and2();

		// Publisher for layered nodes
		max_min_lp_msgs::layered_node_array temp_layered_msg;

		vector<max_min_lp_msgs::layered_node> lay_robot_node = lpc.getRobotLayeredNode();
		vector<max_min_lp_msgs::layered_node> lay_red_node = lpc.getRedLayeredNode();
		vector<max_min_lp_msgs::layered_node> lay_blue_node = lpc.getBlueLayeredNode();
		vector<max_min_lp_msgs::layered_node> lay_target_node = lpc.getTargetLayeredNode();

		for (int i = 0; i < lay_robot_node.size(); i++) {
			temp_layered_msg.lay_nodes.push_back(lay_robot_node[i]);
		}
		for (int i = 0; i < lay_red_node.size(); i++) {
			temp_layered_msg.lay_nodes.push_back(lay_red_node[i]);
		}
		for (int i = 0; i < lay_blue_node.size(); i++) {
			temp_layered_msg.lay_nodes.push_back(lay_blue_node[i]);
		}
		for (int i = 0; i < lay_target_node.size(); i++) {
			temp_layered_msg.lay_nodes.push_back(lay_target_node[i]);
		}

		m_layered_node_pub.publish(temp_layered_msg);
	}
}

bool MaxMinLPRobotNode::initialize() {
	m_client = m_nh.serviceClient<max_min_lp_simulation::MessageRequest>("/robot_request");
	max_min_lp_simulation::MessageRequest srv;
	srv.request.robot_id = m_robot_id;
	srv.request.state_check = "ready";

	// Compute motion primivites.
  	vector<geometry_msgs::Pose> motion_primitive_pose;
	motion_primitive_pose = computeMotionPrimitives();
	srv.request.motion_primitive_info = motion_primitive_pose;

	if (m_client.call(srv)) {
		if (strcmp(srv.response.state_answer.c_str(), "start") == 0) {
			m_max_neighbor_hop = srv.response.max_neighbor_hop;
			m_gen_r_node = srv.response.gen_r_node;
			m_gen_p_r_node = srv.response.gen_p_r_node;
			m_gen_p_t_node = srv.response.gen_p_t_node;
			m_gen_t_node = srv.response.gen_t_node;

			m_num_neighbors_at_each_hop.push_back(srv.response.num_neighbors_at_each_hop);
			m_num_new_targets_at_each_hop.push_back(srv.response.num_new_targets_at_each_hop);

			//// ROBOT info
			// ROBOT to robot
			for (int i = 0; i < srv.response.ROBOT_num_robot.size(); i++) {
				m_ROBOT_num_robot.push_back(srv.response.ROBOT_num_robot[i]);
			}
			for (int i = 0; i < srv.response.prev_accumulate_robot.size(); i++) {
				m_prev_accumulate_robot.push_back(srv.response.prev_accumulate_robot[i]);
			}

			// ROBOT to motion primitive
			for (int i = 0; i < srv.response.ROBOT_num_motion_primitive.size(); i++) {
				m_ROBOT_num_motion_primitive.push_back(srv.response.ROBOT_num_motion_primitive[i]);
			}
			for (int i = 0; i < srv.response.prev_accumulate_motion_primitive.size(); i++) {
				m_prev_accumulate_motion_primitive.push_back(srv.response.prev_accumulate_motion_primitive[i]);
			}

			for (int i = 0; i < srv.response.constraint_value.size(); i++) {
				m_constraint_value.push_back(srv.response.constraint_value[i]);
			}

			return true;
		}
		else {
			return MaxMinLPRobotNode::initialize();
		}
	}
	else {
		ROS_INFO("Fail to communicate with the server. The %s is lost.", m_robot_name.c_str());
		return false;
	}
}

vector<geometry_msgs::Pose> MaxMinLPRobotNode::computeMotionPrimitives() {
	// At this moment, tweak this part. Just consider the case when the number of motion primitives considered is five. This should be changed later.
	// Also, this motion primitives are based on the 1m/s for x-direction and 1rad/s for z-direction.
	vector<geometry_msgs::Pose> temp_motion_primitive;

	// This is hard-coded.
	// int m_motion_case_rotation[m_num_motion_primitive] = {4, 2, 0, -2, -4};
	int m_motion_case_rotation[m_num_motion_primitive] = {2, 0, -2};

	tf::Quaternion q(m_pos.orientation.x, m_pos.orientation.y, m_pos.orientation.z, m_pos.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	double x_new, y_new;

	for (int i = 0; i < m_num_motion_primitive; i++) {
		geometry_msgs::Pose temp_motion_primitive_instance;
		// This is hard-coded.
		if ((yaw + 34 * m_motion_case_rotation[i]) >= 0 && (yaw + 34 * m_motion_case_rotation[i]) < 90) {
			x_new = m_pos.position.x + 0.59 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* cos((yaw + 34 * m_motion_case_rotation[i]) * PHI / 180);
			y_new = m_pos.position.y + 0.59 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* sin((yaw + 34 * m_motion_case_rotation[i]) * PHI / 180);
		}
		else if ((yaw + 34 * m_motion_case_rotation[i]) >= 90 && (yaw + 34 * m_motion_case_rotation[i]) < 180) {
			x_new = m_pos.position.x - 0.59 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* cos((180 - (yaw + 34 * m_motion_case_rotation[i])) * PHI / 180);
			y_new = m_pos.position.y + 0.59 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* sin((180 - (yaw + 34 * m_motion_case_rotation[i])) * PHI / 180);
		}
		else if ((yaw + 34 * m_motion_case_rotation[i]) < 0 && (yaw + 34 * m_motion_case_rotation[i]) >= -90) {
			x_new = m_pos.position.x + 0.59 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* cos((-1) * (yaw + 34 * m_motion_case_rotation[i]) * PHI / 180);
			y_new = m_pos.position.y - 0.59 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* sin((-1) * (yaw + 34 * m_motion_case_rotation[i]) * PHI / 180);
		}
		else if ((yaw + 34 * m_motion_case_rotation[i]) < -90 && (yaw + 34 * m_motion_case_rotation[i]) >= -180) {
			x_new = m_pos.position.x - 0.59 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* cos((180 + (yaw + 34 * m_motion_case_rotation[i])) * PHI / 180);
			y_new = m_pos.position.y - 0.59 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* sin((180 + (yaw + 34 * m_motion_case_rotation[i])) * PHI / 180);
		}

		temp_motion_primitive_instance.position.x = x_new;
		temp_motion_primitive_instance.position.y = y_new;

		if (m_verbal_flag) {
			ROS_INFO("%s: %d'th motion primitiv = (%f, %f)", m_robot_name.c_str(), i+1, x_new, y_new);
		}

		temp_motion_primitive.push_back(temp_motion_primitive_instance);
	}

	return temp_motion_primitive;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_robot_node");

	MaxMinLPRobotNode rn;

	ros::spin();
	return 0;
}