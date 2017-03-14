/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * Important note: in this code a robot is used to take into consideration of the reduction of the step 1 of local algorithm. 
 * Thus, a robot here does not indicate a real robot but an element of a real robot. Instead, ROBOT is used for a real robot.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/27/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPCentralNodeSimulation.hpp"

MaxMinLPCentralNodeSimulation::MaxMinLPCentralNodeSimulation() :
m_num_robot(1), m_num_target(1), m_num_motion_primitive(10), m_num_layer(2), m_objective_option(string("quality_of_tracking")),
 m_fov(10), m_verbal_flag(false), m_verbal_local_flag(false), m_epsilon(0.1), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);
	m_private_nh.getParam("num_layer", m_num_layer);
	m_private_nh.getParam("fov", m_fov);
	m_private_nh.getParam("objective_option", m_objective_option);
	m_private_nh.getParam("verbal_flag", m_verbal_flag);
	m_private_nh.getParam("verbal_local_flag", m_verbal_local_flag);
	m_private_nh.getParam("epsilon", m_epsilon);

	// Services
	m_service = m_nh.advertiseService("/robot_request", &MaxMinLPCentralNodeSimulation::initialize, this);
	m_primitive_service = m_nh.advertiseService("/motion_primitive_request", &MaxMinLPCentralNodeSimulation::sendMotionPrimitive, this);

	m_request_robot_id = 1;
	m_send_robot_id = 1;
	m_check_request_send = true;
	m_check_apply_sequential_send = true;
	m_check_finish_action_apply = 0;

	m_ready_to_send = false;

	m_num_survived_robot = 0;
	m_num_survived_motion_primitive = 0;

	m_time_step = 0;

	m_target_outputFile.open("/home/yoon/yoon/max_min_ws/src/max_min_lp_simulation/data/targets.txt");
	m_outputFile.open("/home/yoon/yoon/max_min_ws/src/max_min_lp_simulation/data/results.txt");

	// Subscribers
	geometry_msgs::Pose dummy_target_pos;
	for (int i = 0; i < m_num_target; i++) {
		m_temp_target_name.push_back("target_"+boost::lexical_cast<string>(i+1));
		m_temp_target_pos.push_back(dummy_target_pos);
	}
	// m_target_1_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 0));
	// m_target_2_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 1));
	// m_target_3_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 2));
	// m_target_4_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 3));
	// m_target_5_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 4));
	// m_target_6_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 5));
	// m_target_7_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 6));
	// m_target_8_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 7));
	// m_target_9_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 8));
	// m_target_10_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 9));
	// m_target_11_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 10));
	// m_target_12_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 11));
	// m_target_13_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 12));
	// m_target_14_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 13));
	// m_target_15_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 14));
	// m_target_16_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 15));
	// m_target_17_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 16));
	// m_target_18_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 17));
	// m_target_19_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 18));
	// m_target_20_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 19));
	// m_target_21_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 20));
	// m_target_22_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 21));
	// m_target_23_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 22));
	// m_target_24_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 23));
	// m_target_25_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 24));
	// m_target_26_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 25));
	// m_target_27_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 26));
	// m_target_28_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 27));
	// m_target_29_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 28));
	// m_target_30_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNodeSimulation::updatePose, this, _1, 29));

	m_comm_graph_by_robots_sub = m_nh.subscribe<std_msgs::String>("/robot_comm_graph", 1000, &MaxMinLPCentralNodeSimulation::applySequentialLocalAlgorithm, this);

	// Publishers
	m_general_node_pub = m_nh.advertise<max_min_lp_msgs::general_node_array>("/central/max_min_lp_msgs/general_node_array", 1);
	m_layered_node_pub = m_nh.advertise<max_min_lp_msgs::layered_node_array>("/central/max_min_lp_msgs/layered_node_array", 1);
	m_response_to_robot_pub = m_nh.advertise<std_msgs::String>("/robot_status", 1);

	ROS_INFO("----------------------------- Time Step %d -----------------------------", m_time_step);
	ROS_INFO(" ");
}

bool MaxMinLPCentralNodeSimulation::initialize(max_min_lp_simulation::MessageRequest::Request &req, max_min_lp_simulation::MessageRequest::Response &res) {
	if (strcmp(req.state_check.c_str(), "ready") == 0) {
		res.state_answer = "wait";

		if (m_request_robot_id == req.robot_id) {
			max_min_lp_msgs::server_to_robots temp_server_to_robots;
			temp_server_to_robots.robot_id = req.robot_id;
			temp_server_to_robots.r_x_pos = req.robot_info.position.x;
			temp_server_to_robots.r_y_pos = req.robot_info.position.y;

			// Obtain motion primitives.
			for (int i = 0; i < m_num_motion_primitive; i++) {
				temp_server_to_robots.primitive_id.push_back(i+1);
				temp_server_to_robots.p_x_pos.push_back(req.motion_primitive_info[i].position.x);
				temp_server_to_robots.p_y_pos.push_back(req.motion_primitive_info[i].position.y);
			}

			ROS_INFO("ROBOT %d : (%.2f, %.2f)", req.robot_id, req.robot_info.position.x, req.robot_info.position.y);
			for (int i = 0; i < m_num_motion_primitive; i++) {
				ROS_INFO("    motion primitive = %d (%.2f, %.2f)", i+1, req.motion_primitive_info[i].position.x, req.motion_primitive_info[i].position.y);
			}

			m_robot_info.each_robot.push_back(temp_server_to_robots);

			if (m_verbal_flag) {
				ROS_INFO("In initialize() of the Central Node");
				ROS_INFO("   ROBOT %d", req.robot_id);
				ROS_INFO("       m_request_robot_id = %d", m_request_robot_id);
				ROS_INFO("       m_send_robot_id = %d", m_send_robot_id);
			}
			
			m_request_robot_id += 1;
			m_send_robot_id = 1;
		}

		// Once all robots gave their local information to the central node, then do the following.
		if (m_request_robot_id == (m_num_robot+1)) {
			// Match relavent motion primitives with targets on the basis of FoV. This happens "only once" in the initialization step.
			if (m_check_request_send) {
				// Initialization
				m_target_id.clear();
				m_target_x_pos.clear();
				m_target_y_pos.clear();
				m_target_observed.clear();
				m_primitive_id.clear();
				m_primitive_original_id.clear();
				m_primitive_x_pos.clear();
				m_primitive_y_pos.clear();
				m_dist_primitive_to_target.clear();
				m_ROBOT_num_robot.clear();
				m_prev_accumulate_robot.clear();
				m_ROBOT_num_motion_primitive.clear();
				m_prev_accumulate_motion_primitive.clear();
				m_ROBOT_neighbor.clear();
				m_ROBOT_assign_targets.clear();
				m_max_neighbor_hop.clear();
				m_robot_id.clear();
				m_robots_to_primitives.clear();
				m_ROBOTs_to_targets.clear();
				m_primitives_to_robots.clear();
				m_primitives_to_targets.clear();
				m_targets_to_primitives.clear();
				m_primitives_to_targets_weight.clear();
				m_targets_to_primitives_weight.clear();
				m_optimal_primitive_id.clear();
				m_optimal_primitive_id_for_plot.clear();
				m_constraint_value.clear();

				// Obtain target information.
				if (m_verbal_flag) {
					ROS_INFO("\nTarget information");
				}

				m_target_odom_client = m_nh.serviceClient<max_min_lp_simulation::GetOdom>("/target_odom_request");
				max_min_lp_simulation::GetOdom srv;
				srv.request.request_odom = string("request");

				if (m_target_odom_client.call(srv)) {
					m_temp_target_pos = srv.response.return_target_odom;
				}
				else {
					ROS_INFO("ERROR: Targets are not property obtained.");
				}

				for (int i = 0; i < m_num_target; i++) {
					m_target_id.push_back(i+1);
					m_target_x_pos.push_back(m_temp_target_pos[i].position.x);
					m_target_y_pos.push_back(m_temp_target_pos[i].position.y);

					if (m_verbal_flag) {
						ROS_INFO("target id = %d, (x, y) = (%f, %f)", i+1, m_temp_target_pos[i].position.x, m_temp_target_pos[i].position.y);
					}
				}

				// Obtain ROBOTs to targets
				for (vector<max_min_lp_msgs::server_to_robots>::iterator it = m_robot_info.each_robot.begin(); it != m_robot_info.each_robot.end(); ++it) {
					vector<int> temp_ROBOTs_to_targets;
					for (int i = 0; i < m_num_target; i++) {
						float dist_ROBOT_to_target = sqrt(pow((m_target_x_pos[i] - it->r_x_pos), 2) + pow((m_target_y_pos[i] - it->r_y_pos), 2));

						if (dist_ROBOT_to_target <= m_fov) {
							temp_ROBOTs_to_targets.push_back(m_target_id[i]);
						}
					}

					m_ROBOTs_to_targets.push_back(temp_ROBOTs_to_targets);
				}

				// Obtain primitives to targets
				// Firstly, see all motion primitives to pair each with targets that are in the FoV.
				if (m_verbal_flag) {
					ROS_INFO("\nPrimitive information");
				}

				for (vector<max_min_lp_msgs::server_to_robots>::iterator it = m_robot_info.each_robot.begin(); it != m_robot_info.each_robot.end(); ++it) {
					int count_num_each_primitives = 0;
					m_prev_accumulate_motion_primitive.push_back(m_num_survived_motion_primitive);

					for (int i = 0; i < it->primitive_id.size(); i++) {
						vector<int> temp_primitives_to_targets;
						vector<float> temp_primitives_to_targets_weight;
						bool check_primitive_exist = false;
						float sum_dist_primitive_to_target = 0;

						for (int j = 0; j < m_target_id.size(); j++) {
							float dist_primitive_to_target = sqrt(pow((m_target_x_pos[j] - it->p_x_pos[i]), 2) + pow((m_target_y_pos[j] - it->p_y_pos[i]), 2));

							if (dist_primitive_to_target <= m_fov && dist_primitive_to_target > 0.3) { // Observed by the corresponding motion primitive. Second condition is for the collision avoidance.
								temp_primitives_to_targets.push_back(m_target_id[j]);

								// Objective option must be taken into account here.
								if (strcmp(m_objective_option.c_str(), "quality_of_tracking") == 0) {
									temp_primitives_to_targets_weight.push_back(dist_primitive_to_target);
								}
								else if (strcmp(m_objective_option.c_str(), "number_of_targets") == 0) {
									temp_primitives_to_targets_weight.push_back(1);
								}

								sum_dist_primitive_to_target += dist_primitive_to_target;
								check_primitive_exist = true;
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

						if (check_primitive_exist) {
							m_primitive_id.push_back(m_num_survived_motion_primitive + 1);
							m_primitive_original_id.push_back(it->primitive_id[i]); // Original id obtained from each robot node.
							m_primitive_x_pos.push_back(it->p_x_pos[i]);
							m_primitive_y_pos.push_back(it->p_y_pos[i]);
							m_dist_primitive_to_target.push_back(sum_dist_primitive_to_target);

							if (m_verbal_flag) {
								ROS_INFO("primitive id = %d, (x, y) = (%.2f, %.2f), original id = %d", m_num_survived_motion_primitive + 1, it->p_x_pos[i], it->p_y_pos[i], it->primitive_id[i]);
							}

							m_primitives_to_targets.push_back(temp_primitives_to_targets);
							m_primitives_to_targets_weight.push_back(temp_primitives_to_targets_weight);
							m_num_survived_motion_primitive += 1;
							count_num_each_primitives += 1;
						}
					}

					m_ROBOT_num_motion_primitive.push_back(count_num_each_primitives);

					if (m_verbal_flag) {
						ROS_INFO("    ROBOT %d : number of motion primitives = %d", it->robot_id, count_num_each_primitives);
					}
				}

				// Obtain robot information.
				// Robots to primitives
				if (m_verbal_flag) {
					ROS_INFO("\nRobot information");
				}

				for (int i = 0; i < m_num_robot; i++) { // m_num_robot = number of ROBOTs
					// 3 cases (1) |V_i|>2, 2) |V_i|=2, and 3) |V_i|=1)
					int count_num_each_robots = 0;
					m_prev_accumulate_robot.push_back(m_num_survived_robot);

					if (m_ROBOT_num_motion_primitive[i] > 2) {
						for (int j = 0; j < m_ROBOT_num_motion_primitive[i]-1; j++) {
							for (int k = j+1; k < m_ROBOT_num_motion_primitive[i]; k++) {
								m_robot_id.push_back(m_num_survived_robot + 1);

								if (m_verbal_flag) {
									ROS_INFO("robot id = %d", m_num_survived_robot + 1);
								}

								vector<int> temp_robots_to_primitives;
								temp_robots_to_primitives.push_back(m_primitive_id[m_prev_accumulate_motion_primitive[i] + j]);
								temp_robots_to_primitives.push_back(m_primitive_id[m_prev_accumulate_motion_primitive[i] + k]);
								m_robots_to_primitives.push_back(temp_robots_to_primitives);

								m_num_survived_robot += 1;
								count_num_each_robots += 1;
							}
						}

						m_ROBOT_num_robot.push_back(count_num_each_robots);
					}
					else if (m_ROBOT_num_motion_primitive[i] == 2) {
						m_robot_id.push_back(m_num_survived_robot + 1);

						if (m_verbal_flag) {
							ROS_INFO("robot id = %d", m_num_survived_robot + 1);
						}

						vector<int> temp_robots_to_primitives;
						temp_robots_to_primitives.push_back(m_primitive_id[m_prev_accumulate_motion_primitive[i]]);
						temp_robots_to_primitives.push_back(m_primitive_id[m_prev_accumulate_motion_primitive[i]] + 1);
						m_robots_to_primitives.push_back(temp_robots_to_primitives);

						m_num_survived_robot += 1;
						count_num_each_robots += 1;

						m_ROBOT_num_robot.push_back(count_num_each_robots);
					}
					else if(m_ROBOT_num_motion_primitive[i] == 1) {
						m_robot_id.push_back(m_num_survived_robot + 1);

						if (m_verbal_flag) {
							ROS_INFO("robot id = %d", m_num_survived_robot + 1);
						}

						vector<int> temp_robots_to_primitives;
						temp_robots_to_primitives.push_back(m_primitive_id[m_prev_accumulate_motion_primitive[i]]);
						m_robots_to_primitives.push_back(temp_robots_to_primitives);

						m_num_survived_robot += 1;
						count_num_each_robots += 1;

						m_ROBOT_num_robot.push_back(count_num_each_robots);
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

				// //// Subgraphs
				// // Generate subgraphs. (At this moment we only consider 1 neighbor hop communication.)
				// vector<vector<int> > observed_targets_to_ROBOTs;
				// for (int i = 0; i < m_target_id.size(); i++) {
				// 	vector<int> temp_observed_targets_to_ROBOTs;
				// 	if (m_target_observed[i]) {
				// 		for (vector<int>::iterator it = m_targets_to_primitives[i].begin(); it != m_targets_to_primitives[i].end(); ++it) {
				// 			for (int j = 0; j < m_num_robot; j++) {
				// 				for (int k = 0; k < m_ROBOT_num_motion_primitive[j]-1; k++) {
				// 					if (m_primitive_id[m_prev_accumulate_motion_primitive[j] + k] == *it) {
				// 						temp_observed_targets_to_ROBOTs.push_back(j+1); // ROBOT id
				// 					}
				// 				}
				// 			}
				// 		}
				// 	}

				// 	observed_targets_to_ROBOTs.push_back(temp_observed_targets_to_ROBOTs);
				// }

				// // Find robot, primitive and target id for each subgraph.
				// vector<vector<int> > subgraphs; // This contains ROBOTs' id
				// for (int i = 0; i < m_num_robot; i++) {
				// 	vector<int> temp_subgraphs;
				// 	for (int j = 0; j < m_target_id.size(); j++) {
				// 		if (m_target_observed[j]) {
				// 			bool check_target_contain_ROBOT = false;
				// 			for (vector<int>::iterator it = observed_targets_to_ROBOTs[j].begin(); it != observed_targets_to_ROBOTs[j].end(); ++it) {
				// 				if (*it == i+1) {
				// 					check_target_contain_ROBOT = true;
				// 				}
				// 			}

				// 			if (check_target_contain_ROBOT) {
				// 				for (vector<int>::iterator it = observed_targets_to_ROBOTs[j].begin(); it != observed_targets_to_ROBOTs[j].end(); ++it) {
				// 					if (*it == i+1) {
				// 						temp_subgraphs.push_back(*it);
				// 					}
				// 				}
				// 			}
				// 		}
				// 	}

				// 	set<int> temp_subgraphs_set(temp_subgraphs.begin(), temp_subgraphs.end());
				// 	temp_subgraphs.clear();
				// 	for (set<int>::iterator it = temp_subgraphs_set.begin(); it != temp_subgraphs_set.end(); ++it) {
				// 		temp_subgraphs.push_back(*it);
				// 	}

				// 	subgraphs.push_back(temp_subgraphs);
				// }

				// Generate a set of general graphs (i.e., number of subgraphs.)


				// Find neighbor hops of each ROBOT and compute the maximum number of neighbor hops.
				// for (int i = 0; i < m_num_robot; i++) {
				// 	int temp_layer_count = 0;
				// 	vector<vector<int> > temp_ROBOT_neighbor;
				// 	vector<int> temp_ROBOT_neighbor_at_layer;
				// 	vector<int> temp_ROBOT_assign_targets;

				// 	vector<int> used_primitives_total;
				// 	vector<int> used_primitives_prev;

				// 	for (int j = m_prev_accumulate_motion_primitive[i]; j < m_prev_accumulate_motion_primitive[i] + m_ROBOT_num_motion_primitive[i]; j++) {
				// 		used_primitives_total.push_back(j);
				// 		used_primitives_prev.push_back(j);
				// 	}

				// 	// This is to obtain unique targets that are observed by ROBOT that we are looking at.
				// 	for (vector<int>::iterator it = used_primitives_prev.begin(); it != used_primitives_prev.end(); ++it) {
				// 		for (vector<int>::iterator itt = m_primitives_to_targets[*it].begin(); itt != m_primitives_to_targets[*it].end(); ++itt) {
				// 			temp_ROBOT_assign_targets.push_back(*itt);
				// 		}
				// 	}
				// 	set<int> temp_ROBOT_assign_targets_set(temp_ROBOT_assign_targets.begin(), temp_ROBOT_assign_targets.end());
				// 	temp_ROBOT_assign_targets.clear();
				// 	for (set<int>::iterator it = temp_ROBOT_assign_targets_set.begin(); it != temp_ROBOT_assign_targets_set.end(); ++it) {
				// 		temp_ROBOT_assign_targets.push_back(*it);
				// 	}
				// 	m_ROBOT_assign_targets.push_back(temp_ROBOT_assign_targets);

				// 	while (1) {
				// 		if (temp_layer_count == m_num_layer) { // The break condition.
				// 			break;
				// 		}

				// 		for (vector<int>::iterator it = temp_ROBOT_neighbor_at_layer.begin(); it != temp_ROBOT_neighbor_at_layer.end(); ++it) {
				// 			for (int j = m_prev_accumulate_motion_primitive[*it-1]; j < m_prev_accumulate_motion_primitive[*it-1] * m_ROBOT_num_motion_primitive[*it-1]; j++) {
				// 				used_primitives_prev.push_back(j);
				// 			}
				// 		}

				// 		temp_ROBOT_neighbor_at_layer.clear();

				// 		vector<int> temp_targets_observed;
				// 		for (vector<int>::iterator it = used_primitives_prev.begin(); it != used_primitives_prev.end(); ++it) {
				// 			for (vector<int>::iterator itt = m_primitives_to_targets[*it].begin(); itt != m_primitives_to_targets[*it].end(); ++itt) {
				// 				temp_targets_observed.push_back(*itt);
				// 			}
				// 		}
				// 		// Sort target IDs observed by ROBOT by finding unique targets.
				// 		set<int> temp_targets_observed_set(temp_targets_observed.begin(), temp_targets_observed.end());
				// 		temp_targets_observed.clear();
				// 		for (set<int>::iterator it = temp_targets_observed_set.begin(); it != temp_targets_observed_set.end(); ++it) {
				// 			temp_targets_observed.push_back(*it);
				// 		}

				// 		vector<int> temp_primitives_linked;
				// 		for (vector<int>::iterator it = temp_targets_observed.begin(); it != temp_targets_observed.end(); ++it) {
				// 			for (vector<int>::iterator itt = m_targets_to_primitives[*it-1].begin(); itt != m_targets_to_primitives[*it-1].end(); ++itt) {
				// 				temp_primitives_linked.push_back(*itt);
				// 			}
				// 		}
				// 		// Sort primitives IDs observed by the corresponding target by finding unique primitives.
				// 		set<int> temp_primitives_linked_set(temp_primitives_linked.begin(), temp_primitives_linked.end());
				// 		temp_primitives_linked.clear();
				// 		for (set<int>::iterator it = temp_primitives_linked_set.begin(); it != temp_primitives_linked_set.end(); ++it) {
				// 			temp_primitives_linked.push_back(*it);
				// 		}

				// 		// Find the neighbor ROBOT IDs. However, this primitive vector contains neighbor's primitives as well as it's own primitives.
				// 		// Thus, do not consider it's own primitives when finding the neighbor ROBOTs.
				// 		for (vector<int>::iterator it = temp_primitives_linked.begin(); it != temp_primitives_linked.end(); ++it) {
				// 			bool check_previous_primitives = false;
				// 			for (vector<int>::iterator itt = used_primitives_total.begin(); itt != used_primitives_total.end(); ++itt) {
				// 				if (*itt == (*it - 1)) {
				// 					check_previous_primitives = true;
				// 					break;
				// 				}
				// 			}
				// 			if (check_previous_primitives) { 
				// 				continue;
				// 			}
				// 			used_primitives_total.push_back((*it - 1));

				// 			int temp_neighbor_ROBOT_id = 0;
				// 			for (int j = 0; j < m_num_robot; j++) {
				// 				if (*it > m_prev_accumulate_motion_primitive[j] && *it <= m_prev_accumulate_motion_primitive[j] + m_ROBOT_num_motion_primitive[j]) {
				// 					temp_neighbor_ROBOT_id = j + 1;
				// 				}
				// 			}
				// 			if (temp_neighbor_ROBOT_id == 0) {
				// 				ROS_INFO("Erorr occurred when finding neighbor ROBOTs.");
				// 				exit (EXIT_FAILURE);
				// 			}

				// 			temp_ROBOT_neighbor_at_layer.push_back(temp_neighbor_ROBOT_id);
				// 		}

				// 		// It is possible that targets observed by the corresponding ROBOT are not observed by any other ROBOTs. 
				// 		// This means that even though the number of layer still needs to be explored, there is no further neighbor hops.
				// 		if (temp_ROBOT_neighbor_at_layer.size() == 0) {
				// 			break;
				// 		}
						
				// 		// Sort ROBOT IDs by finding unique ROBOTs.
				// 		set<int> temp_ROBOT_neighbor_at_layer_set(temp_ROBOT_neighbor_at_layer.begin(), temp_ROBOT_neighbor_at_layer.end());
				// 		temp_ROBOT_neighbor_at_layer.clear();
				// 		for (set<int>::iterator it = temp_ROBOT_neighbor_at_layer_set.begin(); it != temp_ROBOT_neighbor_at_layer_set.end(); ++it) {
				// 			temp_ROBOT_neighbor_at_layer.push_back(*it);
				// 		}

				// 		used_primitives_prev.clear();

				// 		temp_ROBOT_neighbor.push_back(temp_ROBOT_neighbor_at_layer);
				// 		temp_layer_count += 1;
				// 	}

				// 	m_ROBOT_neighbor.push_back(temp_ROBOT_neighbor);
				// 	m_max_neighbor_hop.push_back(temp_layer_count);
				// }

				// Compute contraint values (i.e., 2/|V_i|) for reduction.
				for (vector<int>::iterator it = m_ROBOT_num_motion_primitive.begin(); it != m_ROBOT_num_motion_primitive.end(); ++it) {
					if (*it <= 2) {
						m_constraint_value.push_back(float(1));
					}
					else {
						m_constraint_value.push_back(float(2) / *it);
					}
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

					// ROS_INFO("\n\nNeighbor hops of each ROBOT");
					// for (int i = 0; i < m_num_robot; i++) { // m_primitive_id.size() / m_num_motion_primitive = the number of ROBOTs.
					// 	ROS_INFO("ROBOT id = %d :", i+1);
					// 	ROS_INFO("     number of targets observed = %d", (int)m_ROBOT_assign_targets[i].size());
					// 	for (vector<int>::iterator it = m_ROBOT_assign_targets[i].begin(); it != m_ROBOT_assign_targets[i].end(); ++it) {
					// 		ROS_INFO("     targets observed id = %d", *it);
					// 	}
					// 	ROS_INFO("     layer = 0 :");
					// 	if (m_max_neighbor_hop[i] == 0) {
					// 		continue;
					// 	}
					// 	for (int j = 1; j <= m_num_layer; j++) {
					// 		if (m_max_neighbor_hop[i] >= j) {
					// 			ROS_INFO("     layer = %d :", j);
					// 			for (vector<int>::iterator it = m_ROBOT_neighbor[i][j-1].begin(); it != m_ROBOT_neighbor[i][j-1].end(); ++it) {
					// 				ROS_INFO("          neighbor ROBOT id = %d", *it);
					// 			}
					// 		}
					// 	}
					// }
				}

				m_gen_r_node = buildGeneralNode("r");
				m_gen_p_r_node = buildGeneralNode("p_r");
				m_gen_p_t_node = buildGeneralNode("p_t");
				m_gen_t_node = buildGeneralNode("t");

				m_check_request_send = false;

				m_ROBOT_x_pos.clear();
				m_ROBOT_y_pos.clear();
			} // if (m_check_request_send)

			if (m_send_robot_id == req.robot_id) { // Here send local information to each robot.
				res.state_answer = "start";
				m_send_robot_id += 1;

				// for (int i = 0; i < m_num_robot; i++) {
				// 	if (i == req.robot_id - 1) {
				// 		vector<max_min_lp_msgs::general_node> temp_gen_r_node;
				// 		vector<max_min_lp_msgs::general_node> temp_gen_p_r_node;
				// 		vector<max_min_lp_msgs::general_node> temp_gen_p_t_node;
				// 		vector<max_min_lp_msgs::general_node> temp_gen_t_node;

				// 		vector<int> check_target_overlapped; // This is required because multiple ROBOTs have the same targets.
				// 		int count_new_targets_at_each_hop = 0;

				// 		// Information of the corresponding ROBOT.
				// 		for (int j = m_prev_accumulate_robot[i]; j < m_prev_accumulate_robot[i] + m_ROBOT_num_robot[i]; j++) {
				// 			temp_gen_r_node.push_back(m_gen_r_node[j]);
				// 		}
				// 		for (int j = m_prev_accumulate_motion_primitive[i]; j < m_prev_accumulate_motion_primitive[i] + m_ROBOT_num_motion_primitive[i]; j++) {
				// 			temp_gen_p_r_node.push_back(m_gen_p_r_node[j]);
				// 			temp_gen_p_t_node.push_back(m_gen_p_t_node[j]);
				// 		}
				// 		for (vector<int>::iterator it = m_ROBOT_assign_targets[i].begin(); it != m_ROBOT_assign_targets[i].end(); ++it) {
				// 			check_target_overlapped.push_back(*it);
				// 			temp_gen_t_node.push_back(m_gen_t_node[*it-1]);
				// 			count_new_targets_at_each_hop += 1;
				// 		}

				// 		res.max_neighbor_hop = m_max_neighbor_hop[i];
				// 		res.num_new_targets_at_each_hop = count_new_targets_at_each_hop;

				// 		// Information of neighbor ROBOTs with respect to the corresponding ROBOT.
				// 		if (m_max_neighbor_hop[i] > 0) {
				// 			for (int j = 0; j < m_max_neighbor_hop[i]; j++) { // Maximum number of hops that the corresponding ROBOT can reach.
				// 				res.num_neighbors_at_each_hop = (int)m_ROBOT_neighbor[i][j].size();
				// 				count_new_targets_at_each_hop = 0;

				// 				for (int k = 0; k < m_ROBOT_neighbor[i][j].size(); k++) {
				// 					for (int l = m_prev_accumulate_robot[m_ROBOT_neighbor[i][j][k]-1]; l < m_prev_accumulate_robot[m_ROBOT_neighbor[i][j][k]-1] + m_ROBOT_num_robot[m_ROBOT_neighbor[i][j][k]-1]; l++) {
				// 						temp_gen_r_node.push_back(m_gen_r_node[l]);
				// 					}

				// 					for (int l = m_prev_accumulate_motion_primitive[m_ROBOT_neighbor[i][j][k]-1]; l < m_prev_accumulate_motion_primitive[m_ROBOT_neighbor[i][j][k]-1] + m_ROBOT_num_motion_primitive[m_ROBOT_neighbor[i][j][k]-1]; l++) {
				// 						temp_gen_p_r_node.push_back(m_gen_p_r_node[l]);
				// 						temp_gen_p_t_node.push_back(m_gen_p_t_node[l]);
				// 					}
				// 					for (vector<int>::iterator it = m_ROBOT_assign_targets[m_ROBOT_neighbor[i][j][k]-1].begin(); it != m_ROBOT_assign_targets[m_ROBOT_neighbor[i][j][k]-1].end(); ++it) {
				// 						bool check_previously_included = false;

				// 						for (vector<int>::iterator itt = check_target_overlapped.begin(); itt != check_target_overlapped.end(); ++itt) {
				// 							if (*itt == *it) { // Previously included already. Thus, this target won't be included.
				// 								check_previously_included = true;
				// 							}
				// 						}

				// 						if (!check_previously_included) {
				// 							check_target_overlapped.push_back(*it);
				// 							temp_gen_t_node.push_back(m_gen_t_node[*it-1]);
				// 							count_new_targets_at_each_hop += 1;
				// 						}
				// 					}
				// 				}

				// 				res.num_new_targets_at_each_hop = count_new_targets_at_each_hop;
				// 			}
				// 		}

				// 		// Responses
				// 		res.gen_r_node = temp_gen_r_node;
				// 		res.gen_p_r_node = temp_gen_p_r_node;
				// 		res.gen_p_t_node = temp_gen_p_t_node;
				// 		res.gen_t_node = temp_gen_t_node;

				// 		for (vector<int>::iterator it = m_ROBOT_num_robot.begin(); it != m_ROBOT_num_robot.end(); ++it) {
				// 			res.ROBOT_num_robot.push_back(*it);
				// 		}
				// 		for (vector<int>::iterator it = m_prev_accumulate_robot.begin(); it != m_prev_accumulate_robot.end(); ++it) {
				// 			res.prev_accumulate_robot.push_back(*it);
				// 		}
				// 		res.num_survived_robot = m_num_survived_robot;

				// 		for (vector<int>::iterator it = m_ROBOT_num_motion_primitive.begin(); it != m_ROBOT_num_motion_primitive.end(); ++it) {
				// 			res.ROBOT_num_motion_primitive.push_back(*it);
				// 		}
				// 		for (vector<int>::iterator it = m_prev_accumulate_motion_primitive.begin(); it != m_prev_accumulate_motion_primitive.end(); ++it) {
				// 			res.prev_accumulate_motion_primitive.push_back(*it);
				// 		}
				// 		res.num_survived_motion_primitive = m_num_survived_motion_primitive;

				// 		for (vector<float>::iterator it = m_constraint_value.begin(); it != m_constraint_value.end(); ++it) {
				// 			res.constraint_value.push_back(*it);
				// 		}

				// 		if (m_verbal_flag) {
				// 			ROS_INFO("\n\nROBOT %d's local general graph information.", i + 1);
				// 			ROS_INFO("     maximum number of hops = %d", m_max_neighbor_hop[i]);
				// 			if (m_max_neighbor_hop[i] > 0) {
				// 				for (int j = 0; j < m_max_neighbor_hop[i]; j++) { // Maximum number of hops that the corresponding ROBOT can reach.
				// 					ROS_INFO("          number of neighbors at hop %d = %d", j + 1, (int)m_ROBOT_neighbor[i][j].size());
				// 					for (vector<int>::iterator it = m_ROBOT_neighbor[i][j].begin(); it != m_ROBOT_neighbor[i][j].end(); ++it) {
				// 						ROS_INFO("               neighbor ROBOT = ROBOT %d", *it);
				// 					}
				// 				}
				// 			}
				// 			ROS_INFO("    robot nodes");
				// 			for (vector<max_min_lp_msgs::general_node>::iterator it = temp_gen_r_node.begin(); it != temp_gen_r_node.end(); ++it) {
				// 				ROS_INFO("        type = %s, id = %d, loc_deg = %d", it->type.c_str(), it->id, it->loc_deg);
				// 				for (int j = 0; j < it->loc_deg; j++) {
				// 					ROS_INFO("            loc_id = %d, loc_edge_weight = %f", it->loc_neighbor[j], (float)it->loc_edge_weight[j]);
				// 				}
				// 			}
				// 			ROS_INFO("    red nodes");
				// 			for (vector<max_min_lp_msgs::general_node>::iterator it = temp_gen_p_r_node.begin(); it != temp_gen_p_r_node.end(); ++it) {
				// 				ROS_INFO("        type = %s, id = %d, loc_deg = %d", it->type.c_str(), it->id, it->loc_deg);
				// 				for (int j = 0; j < it->loc_deg; j++) {
				// 					ROS_INFO("            loc_id = %d, loc_edge_weight = %f", it->loc_neighbor[j], (float)it->loc_edge_weight[j]);
				// 				}
				// 			}
				// 			ROS_INFO("    blue nodes");
				// 			for (vector<max_min_lp_msgs::general_node>::iterator it = temp_gen_p_t_node.begin(); it != temp_gen_p_t_node.end(); ++it) {
				// 				ROS_INFO("        type = %s, id = %d, loc_deg = %d", it->type.c_str(), it->id, it->loc_deg);
				// 				for (int j = 0; j < it->loc_deg; j++) {
				// 					ROS_INFO("            loc_id = %d, loc_edge_weight = %f", it->loc_neighbor[j], (float)it->loc_edge_weight[j]);
				// 				}
				// 			}
				// 			ROS_INFO("    target nodes");
				// 			for (vector<max_min_lp_msgs::general_node>::iterator it = temp_gen_t_node.begin(); it != temp_gen_t_node.end(); ++it) {
				// 				ROS_INFO("        type = %s, id = %d, loc_deg = %d", it->type.c_str(), it->id, it->loc_deg);
				// 				for (int j = 0; j < it->loc_deg; j++) {
				// 					ROS_INFO("            loc_id = %d, loc_edge_weight = %f", it->loc_neighbor[j], (float)it->loc_edge_weight[j]);
				// 				}
				// 			}
				// 		}
				// 	}
				// }

				if (m_send_robot_id == (m_num_robot+1)) {
					m_request_robot_id = 1;
				}
			}
		}
	}

	return true;
}

void MaxMinLPCentralNodeSimulation::updatePose(const gazebo_msgs::ModelStates::ConstPtr& msg, int target_id) {
	int size_msg = m_num_robot + m_num_target + 1; // 1 is for 'ground plane' in gazebo.
	int id;
	for (int i = 0; i < size_msg; i++) {
		if (strcmp(msg->name[i].c_str(), m_temp_target_name[target_id].c_str()) == 0) {
			id = i;
		}
	}

	m_temp_target_pos[target_id] = msg->pose[id];
}

vector<max_min_lp_msgs::general_node> MaxMinLPCentralNodeSimulation::buildGeneralNode(string option) {
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
				ROS_INFO("        type = %s, id = %d, loc_deg = %d", "blue", m_primitive_id[i], (int)m_primitives_to_targets[i].size());
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
				ROS_INFO("        type = %s, id = %d, loc_deg = %d", "target", m_target_id[i], (int)m_targets_to_primitives[i].size());
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

void MaxMinLPCentralNodeSimulation::applySequentialLocalAlgorithm(const std_msgs::String::ConstPtr& msg) {
	if (strcmp(msg->data.c_str(), "comm graph is complete") == 0 && m_check_apply_sequential_send) {
		m_check_apply_sequential_send = false;

		//// Publishers
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
		max_min_lp_core::MaxMinLPSequentialCore lps(m_gen_r_node, m_gen_p_r_node, m_gen_p_t_node, m_gen_t_node,
			m_num_layer, m_verbal_local_flag, m_epsilon, m_ROBOT_num_robot, m_prev_accumulate_robot, m_num_survived_robot,
			m_ROBOT_num_motion_primitive, m_prev_accumulate_motion_primitive, m_num_survived_motion_primitive, m_constraint_value);

		// Step 2
		lps.convertSequentialLayeredMaxMinLP();

		//// Publishers
		// Publisher for layered nodes
		max_min_lp_msgs::layered_node_array temp_layered_msg;

		vector<max_min_lp_msgs::layered_node> lay_robot_node = lps.getRobotLayeredNode();
		vector<max_min_lp_msgs::layered_node> lay_red_node = lps.getRedLayeredNode();
		vector<max_min_lp_msgs::layered_node> lay_blue_node = lps.getBlueLayeredNode();
		vector<max_min_lp_msgs::layered_node> lay_target_node = lps.getTargetLayeredNode();

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

		// Step 3 and 4
		lps.applyLocalAlgorithm();

		// Find optimal motion primitives for each robot and send it to them.
		vector<max_min_lp_msgs::general_node> result_values = lps.getXValues();

		for (int i = 0; i < m_num_robot; i++) { // m_num_robot = number of ROBOTs
			vector<int> max_primitive_index;
			float max_z_v_value = 0;
			for (int j = 0; j < m_ROBOT_num_motion_primitive[i]; j++) {
				if (result_values[m_prev_accumulate_motion_primitive[i] + j].z_v >= max_z_v_value) {
					max_z_v_value = result_values[m_prev_accumulate_motion_primitive[i] + j].z_v;
				}
			}

			// Check if there are more than one motion primitive that gives the same maximum z_v.
			for (int j = 0; j < m_ROBOT_num_motion_primitive[i]; j++) {
				if (result_values[m_prev_accumulate_motion_primitive[i] + j].z_v == max_z_v_value) {
					max_z_v_value = result_values[m_prev_accumulate_motion_primitive[i] + j].z_v;
					max_primitive_index.push_back(m_prev_accumulate_motion_primitive[i] + j);
				}
			}

			if (max_primitive_index.size() > 1) { // Find one with the minimum distance to targets.
				// float min_dist_primitive_to_target = 10000;
				int temp_optimal_primitive_id = 0;
				int temp_optimal_primitive_id_for_plot = 0;
				for (vector<int>::iterator it = max_primitive_index.begin(); it != max_primitive_index.end(); ++it) {
					if (m_primitive_original_id[*it] == 2) {
						temp_optimal_primitive_id = m_primitive_original_id[*it];
						temp_optimal_primitive_id_for_plot = m_primitive_id[*it];
					}

					// if (m_dist_primitive_to_target[*it] < min_dist_primitive_to_target) {
					// 	min_dist_primitive_to_target = m_dist_primitive_to_target[*it];
					// 	temp_optimal_primitive_id = m_primitive_original_id[*it];
					// 	temp_optimal_primitive_id_for_plot = m_primitive_id[*it];
					// }
				}

				if (m_verbal_flag) {
					ROS_INFO("temp_optimal_primitive_id = %d", temp_optimal_primitive_id);
					ROS_INFO("temp_optimal_primitive_id_for_plot = %d", temp_optimal_primitive_id_for_plot);
				}

				m_optimal_primitive_id.push_back(temp_optimal_primitive_id);
				m_optimal_primitive_id_for_plot.push_back(temp_optimal_primitive_id_for_plot);
			}
			else { // When there was only one optimal motion primitive.
				m_optimal_primitive_id.push_back(m_primitive_original_id[max_primitive_index[0]]);
				m_optimal_primitive_id_for_plot.push_back(m_primitive_id[max_primitive_index[0]]);
			}

			// ROS_INFO(" ");
			// ROS_INFO("ROBOT %d : selected primitive id = %d", i+1, m_optimal_primitive_id[i]);
			if (m_verbal_flag) {
				ROS_INFO("ROBOT %d : primitive id with maximum z_v = primitive %d", i+1, m_optimal_primitive_id[i]);
			}
		}

		// A greedy approach was attempted to be implemented but not anymore. Just leave a code if we need this greedy algorithm + local algorithm in the future.
		// vector<int> targets_already_considered;
		// for (int i = 0; i < m_num_robot; i++) { // m_num_robot = number of ROBOTs
		// 	vector<int> max_primitive_index;
		// 	float max_z_v_value = 0;
		// 	vector<bool> check_found_optimal;
		// 	bool result_found_optimal = false;
		// 	for (int j = 0; j < m_ROBOT_num_motion_primitive[i]; j++) {
		// 		if (result_values[m_prev_accumulate_motion_primitive[i] + j].z_v == 0.5) {
		// 			check_found_optimal.push_back(false);
		// 		}
		// 		else {
		// 			check_found_optimal.push_back(true);
		// 		}

		// 		if (result_values[m_prev_accumulate_motion_primitive[i] + j].z_v >= max_z_v_value) {
		// 			max_z_v_value = result_values[m_prev_accumulate_motion_primitive[i] + j].z_v;
		// 		}
		// 	}

		// 	for (vector<bool>::iterator it = check_found_optimal.begin(); it != check_found_optimal.end(); ++it) {
		// 		if (*it) {
		// 			result_found_optimal = true;
		// 		}
		// 	}

		// 	if (result_found_optimal) {
		// 		// Check if there are more than one motion primitive that gives the same maximum z_v.
		// 		// If so, a greedy approach is applied.
		// 		for (int j = 0; j < m_ROBOT_num_motion_primitive[i]; j++) {
		// 			if (result_values[m_prev_accumulate_motion_primitive[i] + j].z_v == max_z_v_value) {
		// 				max_z_v_value = result_values[m_prev_accumulate_motion_primitive[i] + j].z_v;
		// 				max_primitive_index.push_back(m_prev_accumulate_motion_primitive[i] + j);
		// 			}
		// 		}

		// 		if (max_primitive_index.size() > 1) { // Find one with the minimum distance to targets.
		// 			float min_dist_primitive_to_target = 10000;
		// 			int temp_optimal_primitive_id = 0;
		// 			for (vector<int>::iterator it = max_primitive_index.begin(); it != max_primitive_index.end(); ++it) {
		// 				if (m_dist_primitive_to_target[*it] < min_dist_primitive_to_target) {
		// 					min_dist_primitive_to_target = m_dist_primitive_to_target[*it];
		// 					temp_optimal_primitive_id = m_primitive_original_id[*it];
		// 				}
		// 			}

		// 			for (vector<int>::iterator it = m_primitives_to_targets[temp_optimal_primitive_id-1].begin(); it != m_primitives_to_targets[temp_optimal_primitive_id-1].end(); ++it) {
		// 				targets_already_considered.push_back(*it);
		// 			}

		// 			m_optimal_primitive_id.push_back(temp_optimal_primitive_id);
		// 		}
		// 		else { // When there was only one optimal motion primitive.
		// 			for (vector<int>::iterator it = m_primitives_to_targets[max_primitive_index[0]].begin(); it != m_primitives_to_targets[max_primitive_index[0]].end(); ++it) {
		// 				targets_already_considered.push_back(*it);
		// 			}

		// 			m_optimal_primitive_id.push_back(m_primitive_original_id[max_primitive_index[0]]);
		// 		}
		// 	}
		// 	else {
		// 		m_optimal_primitive_id.push_back(0); // Greedy approach is required.
		// 	}

		// 	if (m_verbal_flag) {
		// 		ROS_INFO("ROBOT %d : primitive id with maximum z_v = primitive %d", i+1, m_optimal_primitive_id[i]);
		// 	}
		// }

		//// Robot moving
		// If you comment the following line, only one time step is considered. When uncommented, robots move over time.
		m_ready_to_send = true;
	}
	else if (strcmp(msg->data.c_str(), "action is applied") == 0) {
		m_check_finish_action_apply += 1;

		if (m_check_finish_action_apply == m_num_robot) { // All robots finish applying their actions.
			// Writing the results to the text files.
			// int text_index = i+1;
			// string line;
			// string temp_file_path = "/home/yoon/yoon/max_min_ws/src/max_min_lp_demo/data/output_"+boost::lexical_cast<string>(text_index)+".txt";
			// ifstream output_file (temp_file_path.c_str());

			for (int i = 0; i < m_target_id.size(); i++) {
				float temp_target_x_pos = m_target_x_pos[i];
				float temp_target_y_pos = m_target_y_pos[i];

				if (temp_target_x_pos < 0.001 && temp_target_x_pos > 0) {
					temp_target_x_pos = 0;
				}
				if (temp_target_y_pos < 0.001 && temp_target_y_pos > 0) {
					temp_target_y_pos = 0;
				}

				m_target_outputFile<<setprecision(2)<<temp_target_x_pos<<" "<<setprecision(2)<<temp_target_y_pos<<" ";
			}
			m_target_outputFile<<endl;

			// Find all targets observed by any ROBOTs. (Current result)
			ROS_INFO(" ");
			ROS_INFO("Current result");

			vector<int> found_all_targets;
			for (int i = 0; i < m_num_robot; i++) {
				ROS_INFO("ROBOT %d :", i+1);

				for (vector<int>::iterator it = m_ROBOTs_to_targets[i].begin(); it != m_ROBOTs_to_targets[i].end(); ++it) {
					ROS_INFO("    target id = %d", *it);
					found_all_targets.push_back(*it);
				}
			}

			set<int> found_all_targets_set(found_all_targets.begin(), found_all_targets.end());
			found_all_targets.clear();
			for (set<int>::iterator it = found_all_targets_set.begin(); it != found_all_targets_set.end(); ++it) {
				found_all_targets.push_back(*it);
			}

			ROS_INFO("Number of observed targets = %d", (int)found_all_targets.size());

			// Find all targets observed by any motion primitives. (Next result)
			ROS_INFO(" ");
			ROS_INFO("Next result");

			vector<int> found_all_targets_next;
			for (vector<int>::iterator it = m_optimal_primitive_id_for_plot.begin(); it != m_optimal_primitive_id_for_plot.end(); ++it) {
				ROS_INFO("ROBOT %d :", (int)ceil(double(*it) / m_num_motion_primitive));
				for (vector<int>::iterator itt = m_primitives_to_targets[*it-1].begin(); itt != m_primitives_to_targets[*it-1].end(); ++itt) {
					found_all_targets_next.push_back(*itt);
					ROS_INFO("    target id = %d", *itt);
				}
			}

			set<int> found_all_targets_next_set(found_all_targets_next.begin(), found_all_targets_next.end());
			found_all_targets_next.clear();
			for (set<int>::iterator it = found_all_targets_next_set.begin(); it != found_all_targets_next_set.end(); ++it) {
				found_all_targets_next.push_back(*it);
			}

			ROS_INFO("Number of observed targets = %d", (int)found_all_targets_next.size());

			m_outputFile<<(int)found_all_targets_next.size()<<endl;

			m_time_step += 1;
			ROS_INFO(" ");
			ROS_INFO("----------------------------- Step is successively complete -----------------------------");
			ROS_INFO(" ");
			ROS_INFO("----------------------------- Time Step %d -----------------------------", m_time_step);
			ROS_INFO(" ");
			// Reinitiate and recuresively apply the same algorithm that was applied so far.
			// Reset all the values.
			m_request_robot_id = 1;
			m_send_robot_id = 1;
			m_check_request_send = true;
			m_check_apply_sequential_send = true;
			m_check_finish_action_apply = 0;

			m_ready_to_send = false;

			m_num_survived_robot = 0;
			m_num_survived_motion_primitive = 0;

			m_robot_info.each_robot.clear();

			// Send a rostopic on /robot_status so that ROBOTs will compute max min LP again.
			// Publisher
			std_msgs::String msg;
		    stringstream ss;
		    ss<<"recompute";
		    msg.data = ss.str();
		    m_response_to_robot_pub.publish(msg);

		    usleep(1000000);
		}
	}
}

bool MaxMinLPCentralNodeSimulation::sendMotionPrimitive(max_min_lp_simulation::MotionPrimitiveRequest::Request &req, max_min_lp_simulation::MotionPrimitiveRequest::Response &res) {
	if (m_ready_to_send) {
		int count_time_interval = req.count_motion_primitive;
		res.return_count_motion_primitive = count_time_interval + 1;
		res.selected_primitive_id = m_optimal_primitive_id[req.request_ROBOT_id-1];
		return true;
	}
	else {
		return false;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_central_node");

	MaxMinLPCentralNodeSimulation cn;

	ros::spin();

	return 0;
}