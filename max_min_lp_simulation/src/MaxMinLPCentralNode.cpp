/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * Important note: in this code a robot is used to take into consideration of the reduction of the step 1 of local algorithm. 
 * Thus, a robot here does not indicate a real robot but an element of a real robot. Instead, ROBOT is used for a real robot.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/18/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPCentralNode.hpp"

MaxMinLPCentralNode::MaxMinLPCentralNode() :
m_num_robot(1), m_num_target(1), m_num_motion_primitive(10), m_num_layer(2), m_objective_option(string("quality_of_tracking")),
 m_fov(10), m_verbal_flag(false), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);
	m_private_nh.getParam("num_layer", m_num_layer);
	m_private_nh.getParam("fov", m_fov);
	m_private_nh.getParam("objective_option", m_objective_option);
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
			// Match relavent motion primitives with targets on the basis of FoV. This happens only once in the initialization step.
			if (m_check_request_send) {
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
						vector<float> temp_primitives_to_targets_weight;

						for (int j = 0; j < m_target_id.size(); j++) {
							float dist_primitive_to_target = sqrt(pow((m_target_x_pos[j] - it->p_x_pos[i]), 2) + pow((m_target_y_pos[j] - it->p_y_pos[i]), 2));

							if (dist_primitive_to_target <= m_fov) { // Observed by the corresponding motion primitive.
								temp_primitives_to_targets.push_back(m_target_id[j]);

								// Objective option must be taken into account here.
								if (strcmp(m_objective_option.c_str(), "quality_of_tracking") == 0) {
									temp_primitives_to_targets_weight.push_back(dist_primitive_to_target);
								}
								else if (strcmp(m_objective_option.c_str(), "number_of_targets") == 0) {
									temp_primitives_to_targets_weight.push_back(1);
								}
							}
							// else { // Not observed by the corresponding motion primitive.
							// 	if (strcmp(m_objective_option.c_str(), "quality_of_tracking") == 0) {
							// 		temp_primitives_to_targets_weight.push_back(0);
							// 	}
							// 	else if (strcmp(m_objective_option.c_str(), "number_of_targets") == 0) {
							// 		temp_primitives_to_targets_weight.push_back(0);
							// 	}
							// }
						}

						m_primitives_to_targets.push_back(temp_primitives_to_targets);
						m_primitives_to_targets_weight.push_back(temp_primitives_to_targets_weight);
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
						for (vector<int>::iterator it = m_robots_to_primitives[j].begin(); it != m_robots_to_primitives[j].end(); ++it) {
							if (*it == m_primitive_id[i]) {
								temp_primitives_to_robots.push_back(m_robot_id[j]);
							}
						}
					}

					m_primitives_to_robots.push_back(temp_primitives_to_robots);
				}

				// Targets to primitives
				for (int i = 0; i < m_target_id.size(); i++) {
					vector<int> temp_targets_to_primitives;
					vector<float> temp_targets_to_primitives_weight;

					for (int j = 0; j < m_primitive_id.size(); j++) {
						float dist_target_to_primitive = sqrt(pow((m_target_x_pos[i] - m_primitive_x_pos[j]), 2) + pow((m_target_y_pos[i] - m_primitive_y_pos[j]), 2));

						if (dist_target_to_primitive <= m_fov) {
							temp_targets_to_primitives.push_back(m_primitive_id[j]);

							// Objective option must be taken into account here.
							if (strcmp(m_objective_option.c_str(), "quality_of_tracking") == 0) {
								temp_targets_to_primitives_weight.push_back(dist_target_to_primitive);
							}
							else if (strcmp(m_objective_option.c_str(), "number_of_targets") == 0) {
								temp_targets_to_primitives_weight.push_back(1);
							}
						}
						// else { // Not observed by the corresponding motion primitive.
						// 	if (strcmp(m_objective_option.c_str(), "quality_of_tracking") == 0) {
						// 		temp_targets_to_primitives_weight.push_back(0);
						// 	}
						// 	else if (strcmp(m_objective_option.c_str(), "number_of_targets") == 0) {
						// 		temp_targets_to_primitives_weight.push_back(0);
						// 	}
						// }
					}

					m_targets_to_primitives.push_back(temp_targets_to_primitives);
					m_targets_to_primitives_weight.push_back(temp_targets_to_primitives_weight);

					if (temp_targets_to_primitives.size() == 0) {
						m_target_observed.push_back(false);
					}
					else {
						m_target_observed.push_back(true);
					}
				}

				// Find neighbor hops of each ROBOT and compute the maximum number of neighbor hops.
				for (int i = 0; i < m_num_robot; i++) {
					int temp_layer_count = 0;
					vector<vector<int> > temp_ROBOT_neighbor;
					vector<int> temp_ROBOT_neighbor_at_layer;
					vector<int> temp_ROBOT_assign_targets;

					vector<int> used_primitives_total;
					vector<int> used_primitives_prev;

					for (int j = i * m_num_motion_primitive; j < (i + 1) * m_num_motion_primitive; j++) {
						used_primitives_total.push_back(j);
						used_primitives_prev.push_back(j);
					}

					// This is to obtain unique targets that are observed by ROBOT that we are looking at.
					for (vector<int>::iterator it = used_primitives_prev.begin(); it != used_primitives_prev.end(); ++it) {
						for (vector<int>::iterator itt = m_primitives_to_targets[*it].begin(); itt != m_primitives_to_targets[*it].end(); ++itt) {
							temp_ROBOT_assign_targets.push_back(*itt);
						}
					}
					set<int> temp_ROBOT_assign_targets_set(temp_ROBOT_assign_targets.begin(), temp_ROBOT_assign_targets.end());
					temp_ROBOT_assign_targets.clear();
					for (set<int>::iterator it = temp_ROBOT_assign_targets_set.begin(); it != temp_ROBOT_assign_targets_set.end(); ++it) {
						temp_ROBOT_assign_targets.push_back(*it);
					}
					m_ROBOT_assign_targets.push_back(temp_ROBOT_assign_targets);

					while (1) {
						if (temp_layer_count == m_num_layer) { // The break condition.
							break;
						}

						for (vector<int>::iterator it = temp_ROBOT_neighbor_at_layer.begin(); it != temp_ROBOT_neighbor_at_layer.end(); ++it) {
							for (int j = (*it - 1) * m_num_motion_primitive; j < *it * m_num_motion_primitive; j++) {
								used_primitives_prev.push_back(j);
							}
						}

						temp_ROBOT_neighbor_at_layer.clear();

						vector<int> temp_targets_observed;
						for (vector<int>::iterator it = used_primitives_prev.begin(); it != used_primitives_prev.end(); ++it) {
							for (vector<int>::iterator itt = m_primitives_to_targets[*it].begin(); itt != m_primitives_to_targets[*it].end(); ++itt) {
								temp_targets_observed.push_back(*itt);
							}
						}
						// Sort target IDs observed by ROBOT by finding unique targets.
						set<int> temp_targets_observed_set(temp_targets_observed.begin(), temp_targets_observed.end());
						temp_targets_observed.clear();
						for (set<int>::iterator it = temp_targets_observed_set.begin(); it != temp_targets_observed_set.end(); ++it) {
							temp_targets_observed.push_back(*it);
						}

						vector<int> temp_primitives_linked;
						for (vector<int>::iterator it = temp_targets_observed.begin(); it != temp_targets_observed.end(); ++it) {
							for (vector<int>::iterator itt = m_targets_to_primitives[*it-1].begin(); itt != m_targets_to_primitives[*it-1].end(); ++itt) {
								temp_primitives_linked.push_back(*itt);
							}
						}
						// Sort primitives IDs observed by the corresponding target by finding unique primitives.
						set<int> temp_primitives_linked_set(temp_primitives_linked.begin(), temp_primitives_linked.end());
						temp_primitives_linked.clear();
						for (set<int>::iterator it = temp_primitives_linked_set.begin(); it != temp_primitives_linked_set.end(); ++it) {
							temp_primitives_linked.push_back(*it);
						}

						// Find the neighbor ROBOT IDs. However, this primitive vector contains neighbor's primitives as well as it's own primitives.
						// Thus, do not consider it's own primitives when finding the neighbor ROBOTs.
						for (vector<int>::iterator it = temp_primitives_linked.begin(); it != temp_primitives_linked.end(); ++it) {
							bool check_previous_primitives = false;
							for (vector<int>::iterator itt = used_primitives_total.begin(); itt != used_primitives_total.end(); ++itt) {
								if (*itt == (*it - 1)) {
									check_previous_primitives = true;
									break;
								}
							}
							if (check_previous_primitives) { 
								continue;
							}
							used_primitives_total.push_back((*it - 1));
							int temp_neighbor_ROBOT_id = ceil(double(*it) / m_num_motion_primitive);
							temp_ROBOT_neighbor_at_layer.push_back(temp_neighbor_ROBOT_id);
						}

						// It is possible that targets observed by the corresponding ROBOT are not observed by any other ROBOTs. 
						// This means that even though the number of layer still needs to be explored, there is no further neighbor hops.
						if (temp_ROBOT_neighbor_at_layer.size() == 0) {
							break;
						}
						
						// Sort ROBOT IDs by finding unique ROBOTs.
						set<int> temp_ROBOT_neighbor_at_layer_set(temp_ROBOT_neighbor_at_layer.begin(), temp_ROBOT_neighbor_at_layer.end());
						temp_ROBOT_neighbor_at_layer.clear();
						for (set<int>::iterator it = temp_ROBOT_neighbor_at_layer_set.begin(); it != temp_ROBOT_neighbor_at_layer_set.end(); ++it) {
							temp_ROBOT_neighbor_at_layer.push_back(*it);
						}

						used_primitives_prev.clear();

						temp_ROBOT_neighbor.push_back(temp_ROBOT_neighbor_at_layer);
						temp_layer_count += 1;
					}

					m_ROBOT_neighbor.push_back(temp_ROBOT_neighbor);
					m_max_neighbor_hop.push_back(temp_layer_count);
				}

				if (m_verbal_flag) {
					ROS_INFO("\n\nNew target information after the removal");
					for (int i = 0; i < m_target_id.size(); i++) {
						if (m_target_observed[i]) {
							ROS_INFO("target id = %d, (x, y) = (%f, %f)", m_target_id[i], m_target_x_pos[i], m_target_y_pos[i]);
						}
					}

					ROS_INFO("\n\nRobots to primitives");
					for (int i = 0; i < m_robot_id.size(); i++) {
						ROS_INFO("robot id = %d :", m_robot_id[i]);
						int temp_count = 0;
						for (vector<int>::iterator it = m_robots_to_primitives[i].begin(); it != m_robots_to_primitives[i].end(); ++it) {
							temp_count += 1;
							ROS_INFO("          (%d) primitive id = %d", temp_count, *it);
						}
					}

					ROS_INFO("\n\nPrimitives to robots");
					for (int i = 0; i < m_primitive_id.size(); i++) {
						ROS_INFO("primitive id = %d :", m_primitive_id[i]);
						int temp_count = 0;
						for (vector<int>::iterator it = m_primitives_to_robots[i].begin(); it != m_primitives_to_robots[i].end(); ++it) {
							temp_count += 1;
							ROS_INFO("          (%d) robot id = %d", temp_count, *it);
						}
					}

					ROS_INFO("\n\nPrimitives to targets");
					for (int i = 0; i < m_primitive_id.size(); i++) {
						ROS_INFO("primitive id = %d :", m_primitive_id[i]);
						int temp_count = 0;
						for (vector<int>::iterator it = m_primitives_to_targets[i].begin(); it != m_primitives_to_targets[i].end(); ++it) {
							temp_count += 1;
							ROS_INFO("          (%d) target id = %d", temp_count, *it);
						}
					}

					ROS_INFO("\n\nTargets to primitives");
					for (int i = 0; i < m_target_id.size(); i++) {
						if (m_target_observed[i]) {
							ROS_INFO("target id = %d :", m_target_id[i]);
							int temp_count = 0;
							for (vector<int>::iterator it = m_targets_to_primitives[i].begin(); it != m_targets_to_primitives[i].end(); ++it) {
								temp_count += 1;
								ROS_INFO("          (%d) primitive id = %d", temp_count, *it);
							}
						}
					}

					ROS_INFO("\n\nNeighbor hops of each ROBOT");
					for (int i = 0; i < m_primitive_id.size() / m_num_motion_primitive; i++) { // m_primitive_id.size() / m_num_motion_primitive = the number of ROBOTs.
						ROS_INFO("ROBOT id = %d :", i+1);
						ROS_INFO("     number of targets observed = %d", (int)m_ROBOT_assign_targets[i].size());
						for (vector<int>::iterator it = m_ROBOT_assign_targets[i].begin(); it != m_ROBOT_assign_targets[i].end(); ++it) {
							ROS_INFO("     targets observed id = %d", *it);
						}
						ROS_INFO("     layer = 0 :");
						if (m_max_neighbor_hop[i] == 0) {
							continue;
						}
						for (int j = 1; j < m_num_layer; j++) {
							if (m_max_neighbor_hop[i] >= j) {
								ROS_INFO("     layer = %d :", j);
								for (vector<int>::iterator it = m_ROBOT_neighbor[i][j-1].begin(); it != m_ROBOT_neighbor[i][j-1].end(); ++it) {
									ROS_INFO("          neighbor ROBOT id = %d", *it);
								}
							}
						}
					}
				}

				m_gen_r_node = buildGeneralNode("r");
				m_gen_p_r_node = buildGeneralNode("p_r");
				m_gen_p_t_node = buildGeneralNode("p_t");
				m_gen_t_node = buildGeneralNode("t");

				m_check_request_send = false;
			} // if (m_check_request_send)

			if (m_send_robot_id == req.robot_id) { // Here send local information to each robot.
				res.state_answer = "start";
				m_send_robot_id += 1;

				for (int i = 0; i < m_primitive_id.size() / m_num_motion_primitive; i++) {
					if (i == req.robot_id - 1) {
						vector<max_min_lp_msgs::general_node> temp_gen_r_node;
						vector<max_min_lp_msgs::general_node> temp_gen_p_r_node;
						vector<max_min_lp_msgs::general_node> temp_gen_p_t_node;
						vector<max_min_lp_msgs::general_node> temp_gen_t_node;

						vector<int> check_target_overlapped; // This is required because multiple ROBOTs have the same targets.
						int count_new_targets_at_each_hop = 0;

						// Information of the corresponding ROBOT.
						for (int j = i * m_num_constraints; j < (i + 1) * m_num_constraints; j++) {
							temp_gen_r_node.push_back(m_gen_r_node[j]);
						}
						for (int j = i * m_num_motion_primitive; j < (i + 1) * m_num_motion_primitive; j++) {
							temp_gen_p_r_node.push_back(m_gen_p_r_node[j]);
							temp_gen_p_t_node.push_back(m_gen_p_t_node[j]);
						}
						for (vector<int>::iterator it = m_ROBOT_assign_targets[i].begin(); it != m_ROBOT_assign_targets[i].end(); ++it) {
							check_target_overlapped.push_back(*it);
							temp_gen_t_node.push_back(m_gen_t_node[*it-1]);
							count_new_targets_at_each_hop += 1;
						}

						res.max_neighbor_hop = m_max_neighbor_hop[i];
						res.num_new_targets_at_each_hop = count_new_targets_at_each_hop;

						// Information of neighbor ROBOTs with respect to the corresponding ROBOT.
						if (m_max_neighbor_hop[i] > 0) {
							for (int j = 0; j < m_max_neighbor_hop[i]; j++) { // Maximum number of hops that the corresponding ROBOT can reach.
								res.num_neighbors_at_each_hop = (int)m_ROBOT_neighbor[i][j].size();
								count_new_targets_at_each_hop = 0;

								for (int k = 0; k < m_ROBOT_neighbor[i][j].size(); k++) {
									for (int l = (m_ROBOT_neighbor[i][j][k]-1) * m_num_constraints; l < m_ROBOT_neighbor[i][j][k] * m_num_constraints; l++) {
										temp_gen_r_node.push_back(m_gen_r_node[l]);
									}

									for (int l = (m_ROBOT_neighbor[i][j][k]-1) * m_num_motion_primitive; l < m_ROBOT_neighbor[i][j][k] * m_num_motion_primitive; l++) {
										temp_gen_p_r_node.push_back(m_gen_p_r_node[l]);
										temp_gen_p_t_node.push_back(m_gen_p_t_node[l]);
									}
									for (vector<int>::iterator it = m_ROBOT_assign_targets[m_ROBOT_neighbor[i][j][k]-1].begin(); it != m_ROBOT_assign_targets[m_ROBOT_neighbor[i][j][k]-1].end(); ++it) {
										bool check_previously_included = false;

										for (vector<int>::iterator itt = check_target_overlapped.begin(); itt != check_target_overlapped.end(); ++itt) {
											if (*itt == *it) { // Previously included already. Thus, this target won't be included.
												check_previously_included = true;
											}
										}

										if (!check_previously_included) {
											check_target_overlapped.push_back(*it);
											temp_gen_t_node.push_back(m_gen_t_node[*it-1]);
											count_new_targets_at_each_hop += 1;
										}
									}
								}

								res.num_new_targets_at_each_hop = count_new_targets_at_each_hop;
							}
						}

						// Responses
						res.gen_r_node = temp_gen_r_node;
						res.gen_p_r_node = temp_gen_p_r_node;
						res.gen_p_t_node = temp_gen_p_t_node;
						res.gen_t_node = temp_gen_t_node;

						if (m_verbal_flag) {
							ROS_INFO("\n\nROBOT %d's local general graph information.", i + 1);
							ROS_INFO("    maximum number of hops = %d", m_max_neighbor_hop[i]);
							if (m_max_neighbor_hop[i] > 0) {
								for (int j = 0; j < m_max_neighbor_hop[i]; j++) { // Maximum number of hops that the corresponding ROBOT can reach.
									ROS_INFO("      number of neighbors at hop %d = %d", j + 1, (int)m_ROBOT_neighbor[i][j].size());
								}
							}
							ROS_INFO("    robot nodes");
							for (vector<max_min_lp_msgs::general_node>::iterator it = temp_gen_r_node.begin(); it != temp_gen_r_node.end(); ++it) {
								ROS_INFO("        type = %s, id = %d, loc_deg = %d", it->type.c_str(), it->id, it->loc_deg);
								for (int j = 0; j < it->loc_deg; j++) {
									ROS_INFO("            loc_id = %d, loc_edge_weight = %f", it->loc_neighbor[j], (float)it->loc_edge_weight[j]);
								}
							}
							ROS_INFO("    red nodes");
							for (vector<max_min_lp_msgs::general_node>::iterator it = temp_gen_p_r_node.begin(); it != temp_gen_p_r_node.end(); ++it) {
								ROS_INFO("        type = %s, id = %d, loc_deg = %d", it->type.c_str(), it->id, it->loc_deg);
								for (int j = 0; j < it->loc_deg; j++) {
									ROS_INFO("            loc_id = %d, loc_edge_weight = %f", it->loc_neighbor[j], (float)it->loc_edge_weight[j]);
								}
							}
							ROS_INFO("    blue nodes");
							for (vector<max_min_lp_msgs::general_node>::iterator it = temp_gen_p_t_node.begin(); it != temp_gen_p_t_node.end(); ++it) {
								ROS_INFO("        type = %s, id = %d, loc_deg = %d", it->type.c_str(), it->id, it->loc_deg);
								for (int j = 0; j < it->loc_deg; j++) {
									ROS_INFO("            loc_id = %d, loc_edge_weight = %f", it->loc_neighbor[j], (float)it->loc_edge_weight[j]);
								}
							}
							ROS_INFO("    target nodes");
							for (vector<max_min_lp_msgs::general_node>::iterator it = temp_gen_t_node.begin(); it != temp_gen_t_node.end(); ++it) {
								ROS_INFO("        type = %s, id = %d, loc_deg = %d", it->type.c_str(), it->id, it->loc_deg);
								for (int j = 0; j < it->loc_deg; j++) {
									ROS_INFO("            loc_id = %d, loc_edge_weight = %f", it->loc_neighbor[j], (float)it->loc_edge_weight[j]);
								}
							}
						}
					}
				}

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

vector<max_min_lp_msgs::general_node> MaxMinLPCentralNode::buildGeneralNode(string option) {
	vector<max_min_lp_msgs::general_node> gen_return_node;

	if (m_verbal_flag) {
		ROS_INFO("\n\nGeneral nodes information");
	}

	// Robot nodes
	if (strcmp(option.c_str(),"r") == 0) {
		if (m_verbal_flag) {
			ROS_INFO("Building the general node for robots");
		}
		for (int i = 0; i < m_robot_id.size(); i++) {
			max_min_lp_msgs::general_node temp_node;
			temp_node.id = m_robot_id[i];
			temp_node.loc_deg = (int)m_robots_to_primitives[i].size();
			temp_node.type = "robot";
			for (vector<int>::iterator it = m_robots_to_primitives[i].begin(); it != m_robots_to_primitives[i].end(); ++it) {
				temp_node.loc_neighbor.push_back(*it);
				temp_node.loc_edge_weight.push_back(1);
			}

			gen_return_node.push_back(temp_node);

			if (m_verbal_flag) {
				ROS_INFO("        type = %s, id = %d, loc_deg = %d", "robot", m_robot_id[i], (int)m_robots_to_primitives[i].size());
				for (vector<int>::iterator it = m_robots_to_primitives[i].begin(); it != m_robots_to_primitives[i].end(); ++it) {
					ROS_INFO("            loc_id = %d, loc_edge_weight = %f", *it, (float)1);
				}
			}
		}
	}

	// Red nodes
	else if (strcmp(option.c_str(),"p_r") == 0) {
		if (m_verbal_flag) {
			ROS_INFO("Building the general node for primitives to robots (Red nodes)");
		}
		for (int i = 0; i < m_primitive_id.size(); i++) {
			max_min_lp_msgs::general_node temp_node;
			temp_node.id = m_primitive_id[i];
			temp_node.loc_deg = (int)m_primitives_to_robots[i].size();
			temp_node.type = "red";
			for (vector<int>::iterator it = m_primitives_to_robots[i].begin(); it != m_primitives_to_robots[i].end(); ++it) {
				temp_node.loc_neighbor.push_back(*it);
				temp_node.loc_edge_weight.push_back(1);
			}

			gen_return_node.push_back(temp_node);

			if (m_verbal_flag) {
				ROS_INFO("        type = %s, id = %d, loc_deg = %d", "red", m_primitive_id[i], (int)m_primitives_to_robots[i].size());
				for (vector<int>::iterator it = m_primitives_to_robots[i].begin(); it != m_primitives_to_robots[i].end(); ++it) {
					ROS_INFO("            loc_id = %d, loc_edge_weight = %f", *it, (float)1);
				}
			}
		}
	}

	// Blue nodes
	else if (strcmp(option.c_str(),"p_t") == 0) {
		if (m_verbal_flag) {
			ROS_INFO("Building the general node for primitives to targets (Blue nodes)");
		}
		for (int i = 0; i < m_primitive_id.size(); i++) {
			max_min_lp_msgs::general_node temp_node;
			temp_node.id = m_primitive_id[i];
			temp_node.loc_deg = (int)m_primitives_to_targets[i].size();
			temp_node.type = "blue";
			for (vector<int>::iterator it = m_primitives_to_targets[i].begin(); it != m_primitives_to_targets[i].end(); ++it) {
				temp_node.loc_neighbor.push_back(*it);
			}
			for (vector<float>::iterator it = m_primitives_to_targets_weight[i].begin(); it != m_primitives_to_targets_weight[i].end(); ++it) {
				temp_node.loc_edge_weight.push_back(*it);
			}

			gen_return_node.push_back(temp_node);

			if (m_verbal_flag) {
				ROS_INFO("        type = %s, id = %d, loc_deg = %d", "red", m_primitive_id[i], (int)m_primitives_to_targets[i].size());
				for (vector<int>::iterator it = m_primitives_to_targets[i].begin(); it != m_primitives_to_targets[i].end(); ++it) {
					ROS_INFO("            loc_id = %d", *it);
				}
				for (vector<float>::iterator it = m_primitives_to_targets_weight[i].begin(); it != m_primitives_to_targets_weight[i].end(); ++it) {
					ROS_INFO("                         loc_edge_weight = %f", *it);
				}
			}
		}
	}

	// Target nodes
	else if (strcmp(option.c_str(),"t") == 0) {
		if (m_verbal_flag) {
			ROS_INFO("Building the general node for targets");
		}
		for (int i = 0; i < m_target_id.size(); i++) {
			max_min_lp_msgs::general_node temp_node;
			temp_node.id = m_target_id[i];
			temp_node.loc_deg = (int)m_targets_to_primitives[i].size();
			temp_node.type = "target";
			for (vector<int>::iterator it = m_targets_to_primitives[i].begin(); it != m_targets_to_primitives[i].end(); ++it) {
				temp_node.loc_neighbor.push_back(*it);
			}
			for (vector<float>::iterator it = m_targets_to_primitives_weight[i].begin(); it != m_targets_to_primitives_weight[i].end(); ++it) {
				temp_node.loc_edge_weight.push_back(*it);
			}

			gen_return_node.push_back(temp_node);

			if (m_verbal_flag) {
				ROS_INFO("        type = %s, id = %d, loc_deg = %d", "red", m_target_id[i], (int)m_targets_to_primitives[i].size());
				for (vector<int>::iterator it = m_targets_to_primitives[i].begin(); it != m_targets_to_primitives[i].end(); ++it) {
					ROS_INFO("            loc_id = %d", *it);
				}
				for (vector<float>::iterator it = m_targets_to_primitives_weight[i].begin(); it != m_targets_to_primitives_weight[i].end(); ++it) {
					ROS_INFO("                         loc_edge_weight = %f", *it);
				}
			}
		}
	}

	return gen_return_node;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_central_node");

	MaxMinLPCentralNode cn;

	ros::spin();

	return 0;
}