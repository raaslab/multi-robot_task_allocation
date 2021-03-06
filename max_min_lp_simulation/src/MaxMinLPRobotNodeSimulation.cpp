/**
 * Each robot node for the simulation
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/13/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPRobotNodeSimulation.hpp"

#define PHI 3.141592

MaxMinLPRobotNodeSimulation::MaxMinLPRobotNodeSimulation() :
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

	// Output files
	string line;
	string temp_file_path = "/home/yoon/yoon/max_min_ws/src/max_min_lp_simulation/data/local/robots_"+boost::lexical_cast<string>(m_robot_id)+".txt";
	m_robot_outputFile.open(temp_file_path.c_str());

	// Publishers
	m_general_node_pub = m_nh.advertise<max_min_lp_msgs::general_node_array>("/max_min_lp_msgs/general_node_array", 1);
	m_layered_node_pub = m_nh.advertise<max_min_lp_msgs::layered_node_array>("/max_min_lp_msgs/layered_node_array", 1);
	m_response_to_server_pub = m_nh.advertise<std_msgs::String>("/robot_comm_graph", 1);
	m_cmd_vel_robot_pub = m_nh.advertise<geometry_msgs::Twist>("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/cmd_vel_mux/input/teleop", 1);

	// Subscribers
	request_sub = m_nh.subscribe("/robot_status", 1000, &MaxMinLPRobotNodeSimulation::applyMotionPrimitives, this);
	m_odom_sub = m_nh.subscribe("/gazebo/model_states", 1000, &MaxMinLPRobotNodeSimulation::updateOdom, this);

	m_count_initialize_func = 0;

	m_count_time_interval = 0;

	m_check_start = false;

	count_computeMotionPrimitives = 0;

	m_random_number_1 = new int[200];
	m_random_number_2 = new int[200];
	m_random_number_3 = new int[200];
	m_random_number_4 = new int[200];
	m_random_number_5 = new int[200];

	int temp_random_number_1[] = {3,-9,6,-9,-9,3,-3,3,6,3,6,-6,6,12,9,-9,-3,-3,3,3,6,-3,-6,-9,6,-6,-3,0,-6,3,0,-9,6,-9,-6,-6,0,-9,-3,-9,-9,6,-6,3,12,-3,6,6,-3,3,-9,9,-9,-6,6,0,6,-3,-6,-12,3,-3,0,3,-12,-3,6,6,-9,-9,-9,-12,-3,3,6,0,-9,3,-9,-9,-9,-9,-9,-6,-3,-3,-6,-6,9,6,0,-9,-6,-9,9,6,0,-3,-9,3,12,-9,-6,-3,-9,3,-3,12,-3,3,-9,-3,-9,6,9,-3,3,-6,0,9,3,-3,-6,0,-3,-3,0,6,-3,-3,-9,-12,-6,-3,3,12,9,0,-6,6,6,6,6,-9,3,0,-6,-9,9,-9,-9,3,9,0,6,-9,12,0,3,-12,6,6,-9,0,-3,0,-3,-3,-9,-6,-12,9,3,9,-9,9,6,3,0,-6,6,-6,-9,6,3,6,3,-3,-3,9,-3,9,6,9,0,3,12,0,-12,9};
	int temp_random_number_2[] = {9,0,9,6,3,9,9,12,-12,9,3,12,0,0,6,-6,0,9,3,9,6,3,-6,3,-9,3,3,6,9,12,6,3,9,3,-12,-9,9,0,9,-6,0,3,-12,3,-3,-12,0,-6,-9,-6,-9,-6,-12,3,-6,0,6,0,0,0,-9,0,9,9,-6,-6,3,3,-3,-6,12,-9,-9,-9,-9,3,3,-12,9,6,6,-9,9,9,12,9,6,0,-9,-3,-9,-12,12,-6,-6,-3,0,3,-12,9,0,9,-3,0,-12,-9,3,-3,9,-9,12,0,6,12,-6,-3,0,6,9,-9,-9,-3,-12,0,-3,-9,-6,9,3,0,9,-9,6,6,0,-9,3,-6,-9,-6,9,-9,-6,-12,0,-12,9,-6,-9,-6,0,-9,12,-3,-6,-12,-6,-12,0,6,3,-9,-9,6,9,0,-9,9,-3,-6,6,-12,-12,3,3,0,6,6,6,-6,6,0,-3,-12,6,-3,3,6,-9,-9,0,0,9,6,6,-12,-9,-9,6,12};
	int temp_random_number_3[] = {-3,-9,3,0,6,6,3,-12,-9,-3,0,3,-3,9,6,12,0,-3,-9,3,6,-3,-9,-6,-9,-6,0,0,0,9,0,12,3,12,-6,3,-6,3,6,-9,-6,-6,3,9,-3,6,3,-12,3,-3,9,-12,0,-3,0,6,-3,6,0,-12,-9,6,0,-9,-3,3,-6,6,-6,9,-6,6,-6,-6,-9,3,3,0,-3,3,3,3,3,12,-6,6,-6,-9,3,0,0,3,6,-3,3,-3,9,9,-6,3,3,0,9,-6,-3,-9,12,3,0,3,0,3,0,6,0,12,-6,-9,-9,-9,-3,0,-3,6,3,6,9,12,-6,-9,6,-9,0,0,9,0,-3,3,6,0,-3,-9,3,-6,-12,6,-6,0,6,-3,6,-3,3,6,0,-12,-3,-3,-6,-6,9,-3,9,-3,6,-3,6,6,-3,-6,6,12,-3,3,0,9,6,-9,9,12,0,9,3,-9,-6,-3,6,9,6,-3,0,-9,-9,-9,3,0,-6,0,-9,-12};
	int temp_random_number_4[] = {3,-3,12,-6,3,3,-3,-9,-12,-3,-9,6,-3,9,6,3,-9,12,-6,9,-6,-3,-9,3,-9,-12,6,-3,3,-3,3,-12,9,6,6,9,-3,3,3,0,-6,-6,0,-6,6,12,-12,0,-9,6,12,-9,12,-12,3,6,0,9,9,3,-9,-6,-9,-12,-9,3,12,-3,-3,12,12,3,12,6,-3,3,-6,-6,3,0,-3,3,6,3,0,3,0,-9,6,12,-3,12,-3,9,0,-3,-6,-9,-6,6,6,6,-12,9,9,6,-12,-3,6,6,-6,-6,3,0,3,-6,-9,9,6,9,-9,-9,-9,0,-6,9,-9,-12,0,6,-6,-9,-3,-6,0,9,3,-9,-3,-12,0,-3,12,6,0,9,-9,-3,9,9,6,3,-3,9,-9,6,3,9,-3,6,9,-3,0,12,0,-3,3,-3,6,-3,0,6,12,-3,9,6,12,-12,-3,3,-6,-6,6,3,3,3,-12,-3,0,-6,6,9,-6,6,-9,9,-9,3,-3,6};
	int temp_random_number_5[] = {0,0,9,-3,0,12,-12,12,-6,3,3,3,-3,3,6,-12,-9,12,3,-6,-3,-9,-6,-6,-3,-9,-3,-9,9,-9,9,-3,-12,-3,6,6,0,3,9,-12,-6,-12,-6,6,6,9,3,-9,9,6,-6,0,12,6,9,-3,0,0,-6,6,0,3,-6,-9,0,-3,6,6,3,-9,-12,0,-6,12,12,-6,6,9,3,9,12,0,6,3,-12,0,3,0,-3,9,9,9,-3,3,9,9,3,-6,3,-9,-3,3,9,6,0,6,-3,12,12,9,-3,0,-6,6,9,9,0,3,-9,9,0,-6,9,6,9,-6,3,3,-9,-3,-6,6,-6,9,9,-3,0,6,9,3,3,-3,0,6,9,6,-12,3,0,0,-9,9,-3,-6,-3,-3,0,0,-3,-3,0,3,12,6,-3,9,-9,-12,-9,-9,-3,-6,-12,0,-9,-9,3,9,12,3,12,0,0,-3,-3,0,-9,9,-9,-3,9,-3,3,9,9,9,-6,-6,9,3};

	for (int i = 0; i < 200; i++) {
		m_random_number_1[i] = temp_random_number_1[i];
		m_random_number_2[i] = temp_random_number_2[i];
		m_random_number_3[i] = temp_random_number_3[i];
		m_random_number_4[i] = temp_random_number_4[i];
		m_random_number_5[i] = temp_random_number_5[i];
	}
}

void MaxMinLPRobotNodeSimulation::updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg) {
	int size_msg = m_num_robot + m_num_target + 1; // 1 is for 'ground plane' in gazebo.
	int id;

	for (int i = 0; i < size_msg; i++) {
		if (strcmp(msg->name[i].c_str(), m_robot_name.c_str()) == 0) {
			id = i;
		}
	}
	m_pos = msg->pose[id];

	if (m_check_start) {
		// ROS_INFO("ROBOT %d updateOdom() is working.", m_robot_id);
	}
}

void MaxMinLPRobotNodeSimulation::applyMotionPrimitives(const std_msgs::String::ConstPtr& msg) {
	// Write results to output files.
	float temp_robot_x_pos = m_pos.position.x;
	float temp_robot_y_pos = m_pos.position.y;

	if (temp_robot_x_pos < 0.001 && temp_robot_x_pos > 0) {
		temp_robot_x_pos = 0;
	}
	if (temp_robot_y_pos < 0.001 && temp_robot_y_pos > 0) {
		temp_robot_y_pos = 0;
	}

	m_check_start = true;

	m_robot_outputFile<<setprecision(2)<<temp_robot_x_pos<<" "<<setprecision(2)<<temp_robot_y_pos<<" ";
	m_robot_outputFile<<endl;

	bool result_success = initialize();

	if (result_success) {
		// Publisher
		std_msgs::String msg;
	    stringstream ss;
	    ss<<"comm graph is complete";
	    msg.data = ss.str();
	    m_response_to_server_pub.publish(msg);

		if (m_verbal_flag) {
			ROS_INFO("ROBOT %d is initiated.", m_robot_id);
		}

		// // Publisher for general nodes
		// max_min_lp_msgs::general_node_array temp_msg;
		// // Robot nodes
		// for (int i = 0; i < m_gen_r_node.size(); i++) {
		// 	temp_msg.gen_nodes.push_back(m_gen_r_node[i]);
		// }
		// // Motion primitive to robot nodes
		// for (int i = 0; i < m_gen_p_r_node.size(); i++) {
		// 	temp_msg.gen_nodes.push_back(m_gen_p_r_node[i]);
		// }
		// // Motion primitive to target nodes
		// for (int i = 0; i < m_gen_p_t_node.size(); i++) {
		// 	temp_msg.gen_nodes.push_back(m_gen_p_t_node[i]);
		// }
		// // Target nodes
		// for (int i = 0; i < m_gen_t_node.size(); i++) {
		// 	temp_msg.gen_nodes.push_back(m_gen_t_node[i]);
		// }

		// m_general_node_pub.publish(temp_msg);

		// // Local algorithm is applied from here.
		// max_min_lp_core::MaxMinLPDecentralizedCore lpc(m_robot_id, m_gen_r_node, m_gen_p_r_node, m_gen_p_t_node, m_gen_t_node,
		// 	m_num_layer, m_verbal_flag, m_epsilon, m_max_neighbor_hop, m_num_neighbors_at_each_hop, m_ROBOT_num_robot, m_prev_accumulate_robot,
		// 	m_num_survived_robot, m_ROBOT_num_motion_primitive, m_prev_accumulate_motion_primitive, m_num_survived_motion_primitive, m_constraint_value);

		// // Step 2
		// lpc.convertDecentralizedLayeredMaxMinLP();

		// // Step 3
		// //   Phase 1 and 2
		// lpc.applyLocalAlgorithmPhase1and2();

		// // Publisher for layered nodes
		// max_min_lp_msgs::layered_node_array temp_layered_msg;

		// vector<max_min_lp_msgs::layered_node> lay_robot_node = lpc.getRobotLayeredNode();
		// vector<max_min_lp_msgs::layered_node> lay_red_node = lpc.getRedLayeredNode();
		// vector<max_min_lp_msgs::layered_node> lay_blue_node = lpc.getBlueLayeredNode();
		// vector<max_min_lp_msgs::layered_node> lay_target_node = lpc.getTargetLayeredNode();

		// for (int i = 0; i < lay_robot_node.size(); i++) {
		// 	temp_layered_msg.lay_nodes.push_back(lay_robot_node[i]);
		// }
		// for (int i = 0; i < lay_red_node.size(); i++) {
		// 	temp_layered_msg.lay_nodes.push_back(lay_red_node[i]);
		// }
		// for (int i = 0; i < lay_blue_node.size(); i++) {
		// 	temp_layered_msg.lay_nodes.push_back(lay_blue_node[i]);
		// }
		// for (int i = 0; i < lay_target_node.size(); i++) {
		// 	temp_layered_msg.lay_nodes.push_back(lay_target_node[i]);
		// }

		// m_layered_node_pub.publish(temp_layered_msg);

		// Obtain an optimal motion primitive.
		// max_min_lp_simulation::apply_motion_primitive amp(m_num_robot, m_num_target, m_robot_id, m_robot_name, 
		// 	m_motion_primitive_pose[(int)m_motion_primitive_pose.size()-1], m_check_rotation_direction);
		// bool result_move_robot = amp.move_robot();

		m_primitive_client = m_nh.serviceClient<max_min_lp_simulation::MotionPrimitiveRequest>("/motion_primitive_request");
		max_min_lp_simulation::MotionPrimitiveRequest srv;
		srv.request.count_motion_primitive = m_count_time_interval;
		srv.request.request_ROBOT_id = m_robot_id;

		if (m_primitive_client.call(srv)) {
			if (m_verbal_flag) {
				ROS_INFO("ROBOT %d is in the getMotionPrimitive() step", m_robot_id);
			}

			m_count_time_interval = srv.response.return_count_motion_primitive;
			m_selected_primitive_id = srv.response.selected_primitive_id;

			// if (m_verbal_flag) {
				ROS_INFO("ROBOT %d : selected primitive id = %d : (%.2f, %.2f)", m_robot_id, m_selected_primitive_id, m_motion_primitive_pose[m_selected_primitive_id-1].position.x, m_motion_primitive_pose[m_selected_primitive_id-1].position.y);
			// }

			m_move_client = m_nh.serviceClient<max_min_lp_simulation::MoveRobot>("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/move_request", true);
			max_min_lp_simulation::MoveRobot srv;
			srv.request.goal_pos = m_motion_primitive_pose[m_selected_primitive_id-1];
			srv.request.rotation_direction = m_check_rotation_direction[m_selected_primitive_id-1];

			// bool result_get_primitive = getMotionPrimitive();
			// if (result_get_primitive) {
			if (m_move_client.call(srv)) {
				ROS_INFO(" ");
				ROS_INFO("After applying motion primitives.");
				ROS_INFO(" ROBOT %d : (%.2f, %.2f)", m_robot_id, m_pos.position.x, m_pos.position.y);
				std_msgs::String msg;
			    stringstream ss;
			    ss<<"action is applied";
			    msg.data = ss.str();
			    m_response_to_server_pub.publish(msg);

			    if (m_verbal_flag) {
					ROS_INFO("ROBOT %d's action is applied", m_robot_id);
				}
			}
			else {
				// if (m_verbal_flag) {
					ROS_INFO("ERROR: ROBOT %d's action failed.", m_robot_id);
				// }
			}
		}
		else {
			ROS_INFO("ERROR: Fail to communicate with the server. ROBOT %d is lost.", m_robot_id);
			// return MaxMinLPRobotNodeSimulation::initialize();
		}
	}
}

bool MaxMinLPRobotNodeSimulation::initialize() {
	// ros::Duration(0.5).sleep();
	m_client = m_nh.serviceClient<max_min_lp_simulation::MessageRequest>("/robot_request", true);
	max_min_lp_simulation::MessageRequest srv;
	srv.request.robot_id = m_robot_id;
	srv.request.state_check = "ready";
	if (m_verbal_flag) {
		ROS_INFO("ROBOT %d is in the initialize() step", m_robot_id);
	}

	// Send robot info.
	geometry_msgs::Pose robot_pose;
	robot_pose.position.x = m_pos.position.x;
	robot_pose.position.y = m_pos.position.y;
	srv.request.robot_info = robot_pose;

	// Compute motion primivites.
	if (m_count_initialize_func == 0) {
	  	m_motion_primitive_pose.clear();
		m_motion_primitive_pose = computeMotionPrimitives();
	}

	srv.request.motion_primitive_info = m_motion_primitive_pose;

	if (m_client.call(srv)) {
		if (strcmp(srv.response.state_answer.c_str(), "start") == 0) {
			// m_max_neighbor_hop = srv.response.max_neighbor_hop;
			// m_gen_r_node = srv.response.gen_r_node;
			// m_gen_p_r_node = srv.response.gen_p_r_node;
			// m_gen_p_t_node = srv.response.gen_p_t_node;
			// m_gen_t_node = srv.response.gen_t_node;

			// m_num_neighbors_at_each_hop.push_back(srv.response.num_neighbors_at_each_hop);
			// m_num_new_targets_at_each_hop.push_back(srv.response.num_new_targets_at_each_hop);

			// //// ROBOT info
			// // ROBOT to robot
			// for (int i = 0; i < srv.response.ROBOT_num_robot.size(); i++) {
			// 	m_ROBOT_num_robot.push_back(srv.response.ROBOT_num_robot[i]);
			// }
			// for (int i = 0; i < srv.response.prev_accumulate_robot.size(); i++) {
			// 	m_prev_accumulate_robot.push_back(srv.response.prev_accumulate_robot[i]);
			// }

			// // ROBOT to motion primitive
			// for (int i = 0; i < srv.response.ROBOT_num_motion_primitive.size(); i++) {
			// 	m_ROBOT_num_motion_primitive.push_back(srv.response.ROBOT_num_motion_primitive[i]);
			// }
			// for (int i = 0; i < srv.response.prev_accumulate_motion_primitive.size(); i++) {
			// 	m_prev_accumulate_motion_primitive.push_back(srv.response.prev_accumulate_motion_primitive[i]);
			// }

			// for (int i = 0; i < srv.response.constraint_value.size(); i++) {
			// 	m_constraint_value.push_back(srv.response.constraint_value[i]);
			// }

			m_count_initialize_func = 0;
			return true;
		}
		else {
			m_count_initialize_func += 1;
			if (m_verbal_flag) {
				ROS_INFO("ROBOT %d called initizlize() %d times", m_robot_id, m_count_initialize_func);
			}

			return MaxMinLPRobotNodeSimulation::initialize();
		}
	}
	else {
		ROS_INFO("Fail to communicate with the server. ROBOT %d is lost.", m_robot_id);
		return false;
		// return MaxMinLPRobotNodeSimulation::initialize();
	}
}

// bool MaxMinLPRobotNodeSimulation::getMotionPrimitive() {
// 	m_primitive_client = m_nh.serviceClient<max_min_lp_simulation::MotionPrimitiveRequest>("/motion_primitive_request");
// 	max_min_lp_simulation::MotionPrimitiveRequest srv;
// 	srv.request.count_motion_primitive = m_count_time_interval;
// 	srv.request.request_ROBOT_id = m_robot_id;

// 	if (m_primitive_client.call(srv)) {
// 		if (m_verbal_flag) {
// 			ROS_INFO("ROBOT %d is in the getMotionPrimitive() step", m_robot_id);
// 		}

// 		m_count_time_interval = srv.response.return_count_motion_primitive;
// 		m_selected_primitive_id = srv.response.selected_primitive_id;

// 		if (m_verbal_flag) {
// 			ROS_INFO("ROBOT %d accepted a service call (in the getMotionPrimitive() step)", m_robot_id);
// 			ROS_INFO("    ROBOT %d : selected primitive id = %d", m_robot_id, m_selected_primitive_id);
// 		}

// 		// Publish cmd_vel to the ROBOT.
// 		if (m_selected_primitive_id == 1) {
// 			if (m_count_time_interval <= abs(m_motion_case_rotation[m_selected_primitive_id-1])) { // Rotation
// 				geometry_msgs::Twist cmd_vel_msg;

// 				cmd_vel_msg.linear.x = 0;
// 				cmd_vel_msg.linear.y = 0;
// 				cmd_vel_msg.linear.z = 0;
// 				cmd_vel_msg.angular.x = 0;
// 				cmd_vel_msg.angular.y = 0;

// 				if (m_motion_case_rotation[m_selected_primitive_id-1] > 0) {
// 					cmd_vel_msg.angular.z = 0.1;
// 				}
// 				else {
// 					cmd_vel_msg.angular.z = -0.1;	
// 				}

// 				m_cmd_vel_robot_pub.publish(cmd_vel_msg);

// 				// if (m_verbal_flag) {
// 					ROS_INFO("        ROBOT %d : time interval = %d, motion case = %d, cmd_vel = (%.1f, %.1f, %.1f) (%.1f, %.1f, %.1f)", 
// 						m_robot_id, m_count_time_interval, m_motion_case_rotation[m_selected_primitive_id-1], cmd_vel_msg.linear.x, cmd_vel_msg.linear.y,
// 						cmd_vel_msg.linear.z, cmd_vel_msg.angular.x, cmd_vel_msg.angular.y, cmd_vel_msg.angular.z);
// 				// }

// 				usleep(1000000); // Sleeping is required so that robot can move step by step.
// 			}
// 			else { // Moving forward
// 				geometry_msgs::Twist cmd_vel_msg;

// 				cmd_vel_msg.linear.x = 0.1;
// 				cmd_vel_msg.linear.y = 0;
// 				cmd_vel_msg.linear.z = 0;
// 				cmd_vel_msg.angular.x = 0;
// 				cmd_vel_msg.angular.y = 0;
// 				cmd_vel_msg.angular.z = 0;

// 				m_cmd_vel_robot_pub.publish(cmd_vel_msg);

// 				// if (m_verbal_flag) {
// 					ROS_INFO("        ROBOT %d : time interval = %d, motion case = %d, cmd_vel = (%.1f, %.1f, %.1f) (%.1f, %.1f, %.1f)", 
// 						m_robot_id, m_count_time_interval, m_motion_case_rotation[m_selected_primitive_id-1], cmd_vel_msg.linear.x, cmd_vel_msg.linear.y,
// 						cmd_vel_msg.linear.z, cmd_vel_msg.angular.x, cmd_vel_msg.angular.y, cmd_vel_msg.angular.z);
// 				// }

// 				usleep(1000000);
// 			}
// 		}
// 		else {
// 			usleep(1000000);
// 		}

// 		m_odom_client = m_nh.serviceClient<max_min_lp_simulation::GetOdom>("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/odom_request", true);
// 		max_min_lp_simulation::GetOdom srv;
// 		srv.request.request_odom = string("request");

// 		if (m_odom_client.call(srv)) {
// 			geometry_msgs::Pose temp_pos;
// 			temp_pos = srv.response.return_odom;

// 			ROS_INFO("    ROBOT %d : (%.2f, %.2f)", m_robot_id, temp_pos.position.x, temp_pos.position.y);
// 		}

// 		// geometry_msgs::Pose temp_pos;
// 		// max_min_lp_simulation::get_odom gom(m_num_robot, m_num_target, m_robot_id, m_robot_name);
// 		// temp_pos = gom.return_odom();

// 		if (m_count_time_interval == m_time_interval) {
// 			m_count_time_interval = 0;
// 			return true;
// 		}
// 		else {
// 			getMotionPrimitive();
// 		}
// 	}
// 	else {
// 		return getMotionPrimitive();
// 	}
// }

vector<geometry_msgs::Pose> MaxMinLPRobotNodeSimulation::computeMotionPrimitives() {
	// At this moment, tweak this part. Just consider the case when the number of motion primitives considered is five. This should be changed later.
	// Also, this motion primitives are based on the 1m/s for x-direction and 1rad/s for z-direction.
	vector<geometry_msgs::Pose> temp_motion_primitive;

	if (m_verbal_flag) {
		ROS_INFO("ROBOT %d is in the computeMotionPrimitives() step", m_robot_id);
	}

	// This is hard-coded.
	// int m_motion_case_rotation[m_num_motion_primitive] = {4, 2, 0, -2, -4};
	// if ((m_num_motion_primitive % 2) == 0) {
	// 	for (int i = (-1)*(m_num_motion_primitive/2); i <= m_num_motion_primitive/2; i++) {
	// 		if (i == 0) {
	// 			continue;
	// 		}
	// 		m_motion_case_rotation.push_back(i*3);
	// 	}
	// }
	// else {
	// 	for (int i = (-1)*((m_num_motion_primitive-1)/2); i <= (m_num_motion_primitive-1)/2; i++) {
	// 		m_motion_case_rotation.push_back(i*3);
	// 	}
	// }
	// srand (time(NULL));
	// int random_case = rand() % 7 + (-3); // Pick randomly one from -3 to 3
	// random_case *= 3; // From -9 to 9

	// x value: 0.1 m/s * 10 = 0.60 m
	// z value: 0.1 * 10 = 33 (degree)

	int random_case;
	if (m_robot_id == 1) {
		random_case = m_random_number_1[count_computeMotionPrimitives];
	}
	else if (m_robot_id == 2) {
		random_case = m_random_number_2[count_computeMotionPrimitives];
	}
	else if (m_robot_id == 3) {
		random_case = m_random_number_3[count_computeMotionPrimitives];
	}
	else if (m_robot_id == 4) {
		random_case = m_random_number_4[count_computeMotionPrimitives];
	}
	else if (m_robot_id == 5) {
		random_case = m_random_number_5[count_computeMotionPrimitives];
	}

	count_computeMotionPrimitives += 1;

	m_motion_case_rotation.clear();
	// m_motion_case_rotation.push_back(random_case);

	m_motion_case_rotation.push_back(-9);m_motion_case_rotation.push_back(-4);m_motion_case_rotation.push_back(4);m_motion_case_rotation.push_back(9);

	/////////// HERE!!
	// m_motion_case_rotation.push_back(0);m_motion_case_rotation.push_back(-9);m_motion_case_rotation.push_back(-5);m_motion_case_rotation.push_back(5);m_motion_case_rotation.push_back(9);

	tf::Quaternion q(m_pos.orientation.x, m_pos.orientation.y, m_pos.orientation.z, m_pos.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	yaw = yaw * 180 / PHI;

	double x_new, y_new;
	double temp_orientation;

	m_check_rotation_direction.clear();

	/////////// HERE!!
	// for (int i = 0; i < m_num_motion_primitive; i++) {
	// 	if (i == 0) {
	// 		m_check_rotation_direction.push_back(0);
	// 		temp_orientation = yaw;
	// 		if (temp_orientation > 180) {
	// 			temp_orientation -= 360;
	// 		}
	// 		else if (temp_orientation < -180) {
	// 			temp_orientation += 360;
	// 		}

	// 		geometry_msgs::Pose temp_motion_primitive_instance;
	// 		temp_motion_primitive_instance.position.x = m_pos.position.x;
	// 		temp_motion_primitive_instance.position.y = m_pos.position.y;
	// 		temp_motion_primitive_instance.orientation.w = temp_orientation;

	// 		if (m_verbal_flag) {
	// 			ROS_INFO("ROBOT %d : %d'th motion primitiv = (%f, %f)", m_robot_id, i+1, m_pos.position.x, m_pos.position.y);
	// 		}

	// 		temp_motion_primitive.push_back(temp_motion_primitive_instance);
	// 	}
	// 	else if(i > 0) {
	// 		// Check if the rotation is cw, ccw or stationary
	// 		if (m_motion_case_rotation[i] > 0) {
	// 			m_check_rotation_direction.push_back(1);
	// 		}
	// 		else if (m_motion_case_rotation[i] < 0) {
	// 			m_check_rotation_direction.push_back(-1);
	// 		}
	// 		else {
	// 			m_check_rotation_direction.push_back(0);
	// 		}

	// 		geometry_msgs::Pose temp_motion_primitive_instance;
	// 		temp_orientation = yaw + 3.42 * m_motion_case_rotation[i];
	// 		if (temp_orientation > 180) {
	// 			temp_orientation -= 360;
	// 		}
	// 		else if (temp_orientation < -180) {
	// 			temp_orientation += 360;
	// 		}
	// 		// This is hard-coded.
	// 		if (temp_orientation >= 0 && temp_orientation < 90) {
	// 			x_new = m_pos.position.x + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
	// 					* cos(temp_orientation * PHI / 180);
	// 			y_new = m_pos.position.y + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
	// 					* sin(temp_orientation * PHI / 180);
	// 		}
	// 		else if (temp_orientation >= 90 && temp_orientation < 180) {
	// 			x_new = m_pos.position.x - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
	// 					* cos((180 - temp_orientation) * PHI / 180);
	// 			y_new = m_pos.position.y + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
	// 					* sin((180 - temp_orientation) * PHI / 180);
	// 		}
	// 		else if (temp_orientation < 0 && temp_orientation >= -90) {
	// 			x_new = m_pos.position.x + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
	// 					* cos((-1) * temp_orientation * PHI / 180);
	// 			y_new = m_pos.position.y - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
	// 					* sin((-1) * temp_orientation * PHI / 180);
	// 		}
	// 		else if (temp_orientation < -90 && temp_orientation >= -180) {
	// 			x_new = m_pos.position.x - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
	// 					* cos((180 + temp_orientation) * PHI / 180);
	// 			y_new = m_pos.position.y - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
	// 					* sin((180 + temp_orientation) * PHI / 180);
	// 		}

	// 		temp_motion_primitive_instance.position.x = x_new;
	// 		temp_motion_primitive_instance.position.y = y_new;
	// 		temp_motion_primitive_instance.orientation.w = temp_orientation;

	// 		if (m_verbal_flag) {
	// 			ROS_INFO("ROBOT %d : %d'th motion primitiv = (%f, %f)", m_robot_id, i+1, x_new, y_new);
	// 		}

	// 		temp_motion_primitive.push_back(temp_motion_primitive_instance);
	// 	}
	// }

	for (int i = 0; i < m_num_motion_primitive; i++) {
		if (i < m_num_motion_primitive-1) {
			// Check if the rotation is cw, ccw or stationary
			if (m_motion_case_rotation[i] > 0) {
				m_check_rotation_direction.push_back(1);
			}
			else if (m_motion_case_rotation[i] < 0) {
				m_check_rotation_direction.push_back(-1);
			}
			else {
				m_check_rotation_direction.push_back(0);
			}

			geometry_msgs::Pose temp_motion_primitive_instance;
			temp_orientation = yaw + 3.42 * m_motion_case_rotation[i];
			if (temp_orientation > 180) {
				temp_orientation -= 360;
			}
			else if (temp_orientation < -180) {
				temp_orientation += 360;
			}
			// This is hard-coded.
			if (temp_orientation >= 0 && temp_orientation < 90) {
				x_new = m_pos.position.x + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
						* cos(temp_orientation * PHI / 180);
				y_new = m_pos.position.y + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
						* sin(temp_orientation * PHI / 180);
			}
			else if (temp_orientation >= 90 && temp_orientation < 180) {
				x_new = m_pos.position.x - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
						* cos((180 - temp_orientation) * PHI / 180);
				y_new = m_pos.position.y + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
						* sin((180 - temp_orientation) * PHI / 180);
			}
			else if (temp_orientation < 0 && temp_orientation >= -90) {
				x_new = m_pos.position.x + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
						* cos((-1) * temp_orientation * PHI / 180);
				y_new = m_pos.position.y - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
						* sin((-1) * temp_orientation * PHI / 180);
			}
			else if (temp_orientation < -90 && temp_orientation >= -180) {
				x_new = m_pos.position.x - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
						* cos((180 + temp_orientation) * PHI / 180);
				y_new = m_pos.position.y - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
						* sin((180 + temp_orientation) * PHI / 180);
			}

			temp_motion_primitive_instance.position.x = x_new;
			temp_motion_primitive_instance.position.y = y_new;
			temp_motion_primitive_instance.orientation.w = temp_orientation;

			if (m_verbal_flag) {
				ROS_INFO("ROBOT %d : %d'th motion primitiv = (%f, %f)", m_robot_id, i+1, x_new, y_new);
			}

			temp_motion_primitive.push_back(temp_motion_primitive_instance);
		}
		else if(i == m_num_motion_primitive-1) {
			m_check_rotation_direction.push_back(0);
			temp_orientation = yaw;
			if (temp_orientation > 180) {
				temp_orientation -= 360;
			}
			else if (temp_orientation < -180) {
				temp_orientation += 360;
			}

			geometry_msgs::Pose temp_motion_primitive_instance;
			temp_motion_primitive_instance.position.x = m_pos.position.x;
			temp_motion_primitive_instance.position.y = m_pos.position.y;
			temp_motion_primitive_instance.orientation.w = temp_orientation;

			if (m_verbal_flag) {
				ROS_INFO("ROBOT %d : %d'th motion primitiv = (%f, %f)", m_robot_id, i+1, m_pos.position.x, m_pos.position.y);
			}

			temp_motion_primitive.push_back(temp_motion_primitive_instance);
		}
	}

	return temp_motion_primitive;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_robot_node");

	MaxMinLPRobotNodeSimulation rn;

	ros::spin();
	return 0;
}