/**
 * Finds the motion primitive that optimally allocates tracking targets
 * given motion primitives of multiple robots.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \11/22/2016
 * Copyright 2016. All Rights Reserved.
 */

#include "max_min_lp_demo/MaxMinLPDemo.hpp"

#define MAX_VALUE 10000

MaxMinLPDemo::MaxMinLPDemo() :
m_num_layer(2), m_verbal_flag(false), m_epsilon(0.1), m_nh("~")
{
	m_nh.getParam("num_layer", m_num_layer);
	m_nh.getParam("verbal_flag", m_verbal_flag);
	m_nh.getParam("epsilon", m_epsilon);

	// Publishers
	m_general_node_pub = m_nh.advertise<max_min_lp_msgs::general_node_array>("/max_min_lp_msgs/general_node_array", 1);
	m_layered_node_pub = m_nh.advertise<max_min_lp_msgs::layered_node_array>("/max_min_lp_msgs/layered_node_array", 1);

	// Subscribers
	m_test_sub = m_nh.subscribe("/robot_status", 1000, &MaxMinLPDemo::maxMinCallback, this);

	// Initialization
	m_gen_r_node.clear();
	m_gen_p_r_node.clear();
	m_gen_p_t_node.clear();
	m_gen_t_node.clear();

	int temp_deg;
	int temp_num;

	ROS_INFO("Max_min_lp_demo package starts..");

	// Robot nodes
	if (ros::param::get("/max_min_lp_demo_node/num_r", temp_num)) {
	}
	else {
		ROS_WARN("Didn't find num_r");
	}

	for (int i = 0; i < temp_num; i++) {
		max_min_lp_msgs::general_node temp_node;
		vector<int> temp_neighbor;
		vector<float> temp_loc_edge_weight;
		string temp_index = boost::lexical_cast<string>(i+1);
		if (ros::param::get("/max_min_lp_demo_node/r"+temp_index+"_deg", temp_deg)) {
			temp_node.loc_deg = temp_deg;
			temp_node.type = "robot";
			temp_node.id = i+1;

			if (ros::param::get("/max_min_lp_demo_node/r"+temp_index+"_neighbor", temp_neighbor) &&
				ros::param::get("/max_min_lp_demo_node/r"+temp_index+"_edge_weight", temp_loc_edge_weight)) {
				for (int j = 0; j < temp_deg; j++) {
					temp_node.loc_neighbor.push_back(temp_neighbor[j]);
					temp_node.loc_edge_weight.push_back(1/temp_loc_edge_weight[j]);
				}
			}
			else {
				ROS_WARN("Didn't find r%d_neighbor or r%d_edge_weight", i+1, i+1);
			}
		}
		else {
			ROS_WARN("Didn't find r%d_deg", i+1);
		}
		m_gen_r_node.push_back(temp_node);
	}

	// Motion primitive to robot nodes
	if (ros::param::get("/max_min_lp_demo_node/num_p", temp_num)) {
	}
	else {
		ROS_WARN("Didn't find num_p");
	}

	for (int i = 0; i < temp_num; i++) {
		max_min_lp_msgs::general_node temp_node;
		vector<int> temp_neighbor;
		vector<float> temp_loc_edge_weight;
		string temp_index = boost::lexical_cast<string>(i+1);
		if (ros::param::get("/max_min_lp_demo_node/p"+temp_index+"_r_deg", temp_deg)) {
			temp_node.loc_deg = temp_deg;
			temp_node.type = "red";
			temp_node.id = i+1;

			if (ros::param::get("/max_min_lp_demo_node/p"+temp_index+"_r_neighbor", temp_neighbor) &&
				ros::param::get("/max_min_lp_demo_node/p"+temp_index+"_r_edge_weight", temp_loc_edge_weight)) {
				for (int j = 0; j < temp_deg; j++) {
					temp_node.loc_neighbor.push_back(temp_neighbor[j]);
					temp_node.loc_edge_weight.push_back(1/temp_loc_edge_weight[j]);
				}
			}
			else {
				ROS_WARN("Didn't find p%d_r_neighbor or p%d_r_edge_weight", i+1, i+1);
			}
		}
		else {
			ROS_WARN("Didn't find p%d_r_deg", i+1);
		}
		m_gen_p_r_node.push_back(temp_node);
	}

	// Motion primitive to target nodes
	if (ros::param::get("/max_min_lp_demo_node/num_p", temp_num)) {
	}
	else {
		ROS_WARN("Didn't find num_p");
	}

	for (int i = 0; i < temp_num; i++) {
		max_min_lp_msgs::general_node temp_node;
		vector<int> temp_neighbor;
		vector<float> temp_loc_edge_weight;
		string temp_index = boost::lexical_cast<string>(i+1);
		if (ros::param::get("/max_min_lp_demo_node/p"+temp_index+"_t_deg", temp_deg)) {
			temp_node.loc_deg = temp_deg;
			temp_node.type = "blue";
			temp_node.id = i+1;

			if (ros::param::get("/max_min_lp_demo_node/p"+temp_index+"_t_neighbor", temp_neighbor) &&
				ros::param::get("/max_min_lp_demo_node/p"+temp_index+"_t_edge_weight", temp_loc_edge_weight)) {
				for (int j = 0; j < temp_deg; j++) {
					temp_node.loc_neighbor.push_back(temp_neighbor[j]);
					temp_node.loc_edge_weight.push_back(1/temp_loc_edge_weight[j]);
				}
			}
			else {
				ROS_WARN("Didn't find p%d_t_neighbor or p%d_t_edge_weight", i+1, i+1);
			}
		}
		else {
			ROS_WARN("Didn't find p%d_t_deg", i+1);
		}
		m_gen_p_t_node.push_back(temp_node);
	}

	// Target nodes
	if (ros::param::get("/max_min_lp_demo_node/num_t", temp_num)) {
	}
	else {
		ROS_WARN("Didn't find num_t");
	}

	for (int i = 0; i < temp_num; i++) {
		max_min_lp_msgs::general_node temp_node;
		vector<int> temp_neighbor;
		vector<float> temp_loc_edge_weight;
		string temp_index = boost::lexical_cast<string>(i+1);
		if (ros::param::get("/max_min_lp_demo_node/t"+temp_index+"_deg", temp_deg)) {
			temp_node.loc_deg = temp_deg;
			temp_node.type = "target";
			temp_node.id = i+1;

			if (ros::param::get("/max_min_lp_demo_node/t"+temp_index+"_neighbor", temp_neighbor) &&
				ros::param::get("/max_min_lp_demo_node/t"+temp_index+"_edge_weight", temp_loc_edge_weight)) {
				for (int j = 0; j < temp_deg; j++) {
					temp_node.loc_neighbor.push_back(temp_neighbor[j]);
					temp_node.loc_edge_weight.push_back(1/temp_loc_edge_weight[j]);
				}
			}
			else {
				ROS_WARN("Didn't find t%d_neighbor or t%d_edge_weight", i+1, i+1);
			}
		}
		else {
			ROS_WARN("Didn't find t%d_deg", i+1);
		}
		m_gen_t_node.push_back(temp_node);
	}
}

void MaxMinLPDemo::maxMinCallback(const std_msgs::String::ConstPtr& msg) {
	string temp_robot_state = msg->data.c_str();
	if (strcmp(temp_robot_state.c_str(), "arrive") == 0) {
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

		max_min_lp_core::MaxMinLPCore lpc(m_gen_r_node, m_gen_p_r_node, m_gen_p_t_node, m_gen_t_node,
			m_num_layer, m_verbal_flag, m_epsilon);

		// Step 2
		lpc.convertLayeredMaxMinLP();
		// Step 3
		lpc.applyLocalAlgorithm();

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

int main(int argc, char** argv) {
	ros::init(argc, argv, "max_min_lp_demo");
	MaxMinLPDemo lp;

	ros::spin();
	return 0;
}
