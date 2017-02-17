/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/10/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPCentralNode.hpp"

MaxMinLPCentralNode::MaxMinLPCentralNode() :
m_num_robot(1), m_num_target(1), m_fov(10), m_num_motion_primitive(10), m_verbal_flag(false), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("fov", m_fov);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);
	m_private_nh.getParam("verbal_flag", m_verbal_flag);

	// // Services
	m_service = m_nh.advertiseService("/robot_request", &MaxMinLPCentralNode::initialize, this);

	m_request_robot_id = 1;
	m_send_robot_id = 1;
	m_check_request_send = true;

	m_num_constraints = 1;
	m_constraint_value = 1;

	// // Subscribers
	geometry_msgs::Pose dummy_target_pos;
	for (int i = 0; i < m_num_target; i++) {
		m_temp_target_name.push_back("target_"+boost::lexical_cast<string>(i+1));
		m_temp_target_pos.push_back(dummy_target_pos);
	}
	m_target_1_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNode::updatePose, this, _1, 0));
	m_target_2_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNode::updatePose, this, _1, 1));
	// m_target_3_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNode::updatePose, this, _1, 2));
}

bool MaxMinLPCentralNode::initialize(max_min_lp_simulation::MessageRequest::Request &req, max_min_lp_simulation::MessageRequest::Response &res) {
	if (strcmp(req.state_check.c_str(), "ready") == 0) {
		if (m_request_robot_id == req.robot_id) {
			max_min_lp_msgs::server_to_robots temp_server_to_robots;
			temp_server_to_robots.robot_id = req.robot_id;

			// Obtain motion primitives.
			for (int i = 0; i < m_num_motion_primitive; i++) {
				temp_server_to_robots.primitive_id.push_back(i+1);
				temp_server_to_robots.p_x_pos.push_back(req.motion_primitive_info[i].position.x);
				temp_server_to_robots.p_y_pos.push_back(req.motion_primitive_info[i].position.y);
			}

			m_robot_info.each_robot.push_back(temp_server_to_robots);
			
			res.state_answer = "wait";
			m_request_robot_id += 1;
			m_send_robot_id = 1;
		}

		// Once all robots gave their local information to the central node, then do the following.
		if (m_request_robot_id == (m_num_robot+1)) {
			if (m_check_request_send) { // Match relavent motion primitives with targets on the basis of FoV. This happens only once in the initialization step.
				if (m_num_motion_primitive > 2) {
					m_num_constraints = req.num_constraints;
					m_constraint_value = req.constraint_value;
				}
				// Obtain target information.
				if (m_verbal_flag) {
					ROS_INFO("\nTarget information");
				}
				for (int i = 0; i < m_num_target; i++) {
					m_target_id.push_back(i+1);
					m_target_x_pos.push_back(m_temp_target_pos[i].position.x);
					m_target_y_pos.push_back(m_temp_target_pos[i].position.y);

					if (m_verbal_flag) {
						ROS_INFO("target id = %d, (x, y) = (%f, %f)", i+1, m_temp_target_pos[i].position.x, m_temp_target_pos[i].position.y);
					}
				}

				// Obtain primitives to targets
				// Firstly, see all motion primitives to pair each with targets that are in the FoV.
				if (m_verbal_flag) {
					ROS_INFO("\nPrimitive information");
				}

				int count_num_total_primitives = 0;
				for (vector<max_min_lp_msgs::server_to_robots>::iterator it = m_robot_info.each_robot.begin(); it != m_robot_info.each_robot.end(); ++it) {
					for (int i = 0; i < it->primitive_id.size(); i++) {
						m_primitive_id.push_back(count_num_total_primitives + 1);
						m_primitive_x_pos.push_back(it->p_x_pos[i]);
						m_primitive_y_pos.push_back(it->p_y_pos[i]);

						if (m_verbal_flag) {
							ROS_INFO("primitive id = %d, (x, y) = (%f, %f)", count_num_total_primitives + 1, it->p_x_pos[i], it->p_y_pos[i]);
						}

						vector<int> temp_primitives_to_targets;

						for (int j = 0; j < m_target_id.size(); j++) {
							float dist_primitive_to_target = sqrt(pow((m_target_x_pos[j] - it->p_x_pos[i]), 2) + pow((m_target_y_pos[j] - it->p_y_pos[i]), 2));

							if (dist_primitive_to_target <= m_fov) {
								temp_primitives_to_targets.push_back(m_target_id[j]);
							}
						}

						m_primitives_to_targets.push_back(temp_primitives_to_targets);
						count_num_total_primitives += 1;
					}
				}

				// Obtain robot information.
				// Robots to primitives
				if (m_verbal_flag) {
					ROS_INFO("\nRobot information");
				}
				int count_num_total_robot = 0;
				if (m_num_motion_primitive > 2) {
					for (int i = 0; i < m_primitive_id.size(); i += m_num_motion_primitive) {
						for (int j = 0; j < m_num_motion_primitive-1; j++) {
							for (int k = j+1; k < m_num_motion_primitive; k++) {
								m_robot_id.push_back(count_num_total_robot + 1);

								if (m_verbal_flag) {
									ROS_INFO("robot id = %d", count_num_total_robot + 1);
								}

								vector<int> temp_robots_to_primitives;
								temp_robots_to_primitives.push_back(m_primitive_id[i + j]);
								temp_robots_to_primitives.push_back(m_primitive_id[i + k]);
								m_robots_to_primitives.push_back(temp_robots_to_primitives);

								count_num_total_robot += 1;
							}
						}

						// Check if the result of combination is correct or not.
						if (i == 0) {
							if (count_num_total_robot != m_num_constraints) {
								cout<<"ERROR: the computation of the combination is wrong."<<endl;
								exit (EXIT_FAILURE);
							}
						}
					}
				}
				else { // When m_num_motion_primitive = 2.
					for (int i = 0; i < m_primitive_id.size(); i += m_num_motion_primitive) {
						m_robot_id.push_back(count_num_total_robot + 1);

						if (m_verbal_flag) {
							ROS_INFO("robot id = %d", count_num_total_robot + 1);
						}

						vector<int> temp_robots_to_primitives;
						temp_robots_to_primitives.push_back(m_primitive_id[i]);
						temp_robots_to_primitives.push_back(m_primitive_id[i + 1]);
						m_robots_to_primitives.push_back(temp_robots_to_primitives);

						count_num_total_robot += 1;
					}
				}

				// Primitives to robots
				for (int i = 0; i < m_primitive_id.size(); i++) {
					vector<int> temp_primitives_to_robots;

					for (int j = 0; j < m_robot_id.size(); j++) {
						for (vector<int>::iterator itt = m_robots_to_primitives[j].begin(); itt != m_robots_to_primitives[j].end(); ++itt) {
							if (*itt == m_primitive_id[i]) {
								temp_primitives_to_robots.push_back(m_robot_id[j]);
							}
						}
					}

					m_primitives_to_robots.push_back(temp_primitives_to_robots);
				}

				// Targets to primitives
				vector<int> remove_target_index;
				for (int i = 0; i < m_target_id.size(); i++) {
					bool check_target_observed = false;
					vector<int> temp_targets_to_primitives;
					for (int j = 0; j < m_primitive_id.size(); j++) {
						float dist_target_to_primitive = sqrt(pow((m_target_x_pos[i] - m_primitive_x_pos[j]), 2) + pow((m_target_y_pos[i] - m_primitive_y_pos[j]), 2));

						if (dist_target_to_primitive <= m_fov) {
							temp_targets_to_primitives.push_back(m_primitive_id[j]);
						}
					}

					if (temp_targets_to_primitives.size() != 0) {
						m_targets_to_primitives.push_back(temp_targets_to_primitives);
						check_target_observed = true;
					}

					// By removing unobserved targets we can get a normalized graph.
					if (!check_target_observed) {
						remove_target_index.push_back(i);
					}
				}

				// Remove unobserved targets from target-related vectors.
				if (remove_target_index.size() != 0) {
					for (vector<int>::reverse_iterator it = remove_target_index.rbegin(); it != remove_target_index.rend(); ++it) { // Remove elements of vector container from the end.
						m_target_id.erase(m_target_id.begin() + *it);
						m_target_x_pos.erase(m_target_x_pos.begin() + *it);
						m_target_y_pos.erase(m_target_y_pos.begin() + *it);
					}
				}

				if (m_verbal_flag) {
					ROS_INFO("\nNew target information after the removal");
					for (int i = 0; i < m_target_id.size(); i++) {
						ROS_INFO("target id = %d, (x, y) = (%f, %f)", m_target_id[i], m_target_x_pos[i], m_target_y_pos[i]);
					}
				}

				if (m_verbal_flag) {
					ROS_INFO("\nRobots to primitives");
					for (int i = 0; i < m_robot_id.size(); i++) {
						ROS_INFO("robot id = %d :", m_robot_id[i]);
						int temp_count = 0;
						for (vector<int>::iterator it = m_robots_to_primitives[i].begin(); it != m_robots_to_primitives[i].end(); ++it) {
							temp_count += 1;
							ROS_INFO("          (%d) primitive id = %d", temp_count, *it);
						}
					}

					ROS_INFO("\nPrimitives to robots");
					for (int i = 0; i < m_primitive_id.size(); i++) {
						ROS_INFO("primitive id = %d :", m_primitive_id[i]);
						int temp_count = 0;
						for (vector<int>::iterator it = m_primitives_to_robots[i].begin(); it != m_primitives_to_robots[i].end(); ++it) {
							temp_count += 1;
							ROS_INFO("          (%d) robot id = %d", temp_count, *it);
						}
					}

					ROS_INFO("\nPrimitives to targets");
					for (int i = 0; i < m_primitive_id.size(); i++) {
						ROS_INFO("primitive id = %d :", m_primitive_id[i]);
						int temp_count = 0;
						for (vector<int>::iterator it = m_primitives_to_targets[i].begin(); it != m_primitives_to_targets[i].end(); ++it) {
							temp_count += 1;
							ROS_INFO("          (%d) target id = %d", temp_count, *it);
						}
					}

					ROS_INFO("\nTargets to primitives");
					for (int i = 0; i < m_target_id.size(); i++) {
						ROS_INFO("target id = %d :", m_target_id[i]);
						int temp_count = 0;
						for (vector<int>::iterator it = m_targets_to_primitives[i].begin(); it != m_targets_to_primitives[i].end(); ++it) {
							temp_count += 1;
							ROS_INFO("          (%d) primitive id = %d", temp_count, *it);
						}
					}
				}

				// Get ready to send local information to each corresponding robot using the above relationships between motion primitives and targets.
				// for (vector<max_min_lp_msgs::server_to_robots>::iterator it = m_robot_info.each_robot.begin(); it != m_robot_info.each_robot.end(); ++it) { // For each robot
				// 	for(int i = 0; i < it->primitive_id.size(); i++) { // For each motion primitive
				// 		if (m_primitives_to_targets[i].size() == 0) { // No targets are connected to this motion primitive.
				// 			it->target_exist.push_back(false);
				// 		}
				// 		else { // Targets are connected to this motion primitive.
				// 			it->target_exist.push_back(true);
				// 			max_min_lp_msgs::target_node temp_target_node;
							
				// 			for (vector<int>::iterator itt = m_primitives_to_targets[i].begin(); itt != m_primitives_to_targets[i].end(); ++itt) { // For each target
				// 				temp_target_node.target_id.push_back(*itt);
				// 				temp_target_node.t_x_pos.push_back(m_target_x_pos[*itt - 1]);
				// 				temp_target_node.t_y_pos.push_back(m_target_y_pos[*itt - 1]);

				// 				max_min_lp_msgs::primitive_node temp_primitive_node;

				// 				if (m_targets_to_primitives[*itt - 1].size() == 0) { // No neighbor motion primitives are connected to this target.
				// 					temp_target_node.neighbor_primitive_exist.push_back(false);
				// 				}
				// 				else {
				// 					temp_target_node.neighbor_primitive_exist.push_back(true);

				// 					for (vector<int>::iterator ittt = m_targets_to_primitives[*itt - 1].begin(); ittt != m_targets_to_primitives[*itt - 1].end(); ++ittt) { // For each connected neighbor motion primitive
				// 						if (*ittt == it->primitive_id[i]) { // Array of targets to primitives include all possible primitives so the primitive we are looking at now must not be taken into consideration.
				// 							continue;
				// 						}
				// 						temp_primitive_node.neighbor_primitive_id.push_back(*ittt);
				// 						temp_primitive_node.neighbor_p_x_pos.push_back(m_primitive_x_pos[*ittt - 1]);
				// 						temp_primitive_node.neighbor_p_y_pos.push_back(m_primitive_y_pos[*ittt - 1]);
				// 					}
				// 				}

				// 				temp_target_node.connect_primitive.push_back(temp_primitive_node);
				// 			}

				// 			it->connect_target.push_back(temp_target_node);
				// 		}
				// 	}
				// }

				m_check_request_send = false;
			} // if (m_check_request_send)

			if (m_send_robot_id == req.robot_id) { // Here send local information to each robot.
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

void MaxMinLPCentralNode::updatePose(const gazebo_msgs::ModelStates::ConstPtr& msg, int target_id) {
	int size_msg = m_num_robot + m_num_target + 1; // 1 is for 'ground plane' in gazebo.
	int id;
	for (int i = 0; i < size_msg; i++) {
		if (strcmp(msg->name[i].c_str(), m_temp_target_name[target_id].c_str()) == 0) {
			id = i;
		}
	}

	m_temp_target_pos[target_id] = msg->pose[id];
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_central_node");

	MaxMinLPCentralNode cn;

	ros::spin();

	return 0;
}