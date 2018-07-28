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
m_input_type(string("yaml")), m_num_layer(2), m_num_text_files(1), m_verbal_flag(false), m_epsilon(0.1), m_nh("~")
{
	m_nh.getParam("input_type", m_input_type);
	m_nh.getParam("num_layer", m_num_layer);
	m_nh.getParam("num_text_files", m_num_text_files);
	m_nh.getParam("verbal_flag", m_verbal_flag);
	m_nh.getParam("epsilon", m_epsilon);

	// Publishers
	m_general_node_pub = m_nh.advertise<max_min_lp_msgs::general_node_array>("/max_min_lp_msgs/general_node_array", 1);
	m_layered_node_pub = m_nh.advertise<max_min_lp_msgs::layered_node_array>("/max_min_lp_msgs/layered_node_array", 1);

	// Subscribers
	m_test_sub = m_nh.subscribe("/robot_status", 1000, &MaxMinLPDemo::maxMinCallback, this);

	m_count = 0;
	check_start = true;
	string partial_path;
	m_nh.getParam("output_path", partial_path);
	m_output_type = partial_path+"output.txt";
	m_outputFile.open(m_output_type.c_str());
	m_output_type_journal = partial_path+"output_journal.txt";
	m_outputFile_journal.open(m_output_type_journal.c_str());
}

void MaxMinLPDemo::maxMinCallback(const std_msgs::String::ConstPtr& msg) {
	if (strcmp(msg->data.c_str(), "demo") == 0 && check_start) {
		check_start = false;
		// Initialization
		m_gen_r_node.clear();
		m_gen_p_r_node.clear();
		m_gen_p_t_node.clear();
		m_gen_t_node.clear();

		int temp_deg;
		int temp_num;

		if (m_verbal_flag) {
			ROS_INFO("Max_min_lp_demo package starts..");
		}

		if (strcmp(m_input_type.c_str(),"yaml") == 0) {
			// Robot nodes
			if (ros::param::get("max_min_lp_demo_node/num_r", temp_num)) {
			}
			else {
				ROS_WARN("Didn't find num_r");
			}

			for (int i = 0; i < temp_num; i++) {
				max_min_lp_msgs::general_node temp_node;
				vector<int> temp_neighbor;
				vector<float> temp_loc_edge_weight;
				string temp_index = boost::lexical_cast<string>(i+1);
				if (ros::param::get("max_min_lp_demo_node/r"+temp_index+"_deg", temp_deg)) {
					temp_node.loc_deg = temp_deg;
					temp_node.type = "robot";
					temp_node.id = i+1;

					if (ros::param::get("max_min_lp_demo_node/r"+temp_index+"_neighbor", temp_neighbor) &&
						ros::param::get("max_min_lp_demo_node/r"+temp_index+"_edge_weight", temp_loc_edge_weight)) {
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
			if (ros::param::get("max_min_lp_demo_node/num_p", temp_num)) {
			}
			else {
				ROS_WARN("Didn't find num_p");
			}

			for (int i = 0; i < temp_num; i++) {
				max_min_lp_msgs::general_node temp_node;
				vector<int> temp_neighbor;
				vector<float> temp_loc_edge_weight;
				string temp_index = boost::lexical_cast<string>(i+1);
				if (ros::param::get("max_min_lp_demo_node/p"+temp_index+"_r_deg", temp_deg)) {
					temp_node.loc_deg = temp_deg;
					temp_node.type = "red";
					temp_node.id = i+1;

					if (ros::param::get("max_min_lp_demo_node/p"+temp_index+"_r_neighbor", temp_neighbor) &&
						ros::param::get("max_min_lp_demo_node/p"+temp_index+"_r_edge_weight", temp_loc_edge_weight)) {
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
			if (ros::param::get("max_min_lp_demo_node/num_p", temp_num)) {
			}
			else {
				ROS_WARN("Didn't find num_p");
			}

			for (int i = 0; i < temp_num; i++) {
				max_min_lp_msgs::general_node temp_node;
				vector<int> temp_neighbor;
				vector<float> temp_loc_edge_weight;
				string temp_index = boost::lexical_cast<string>(i+1);
				if (ros::param::get("max_min_lp_demo_node/p"+temp_index+"_t_deg", temp_deg)) {
					temp_node.loc_deg = temp_deg;
					temp_node.type = "blue";
					temp_node.id = i+1;

					if (ros::param::get("max_min_lp_demo_node/p"+temp_index+"_t_neighbor", temp_neighbor) &&
						ros::param::get("max_min_lp_demo_node/p"+temp_index+"_t_edge_weight", temp_loc_edge_weight)) {
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
			if (ros::param::get("max_min_lp_demo_node/num_t", temp_num)) {
			}
			else {
				ROS_WARN("Didn't find num_t");
			}

			for (int i = 0; i < temp_num; i++) {
				max_min_lp_msgs::general_node temp_node;
				vector<int> temp_neighbor;
				vector<float> temp_loc_edge_weight;
				string temp_index = boost::lexical_cast<string>(i+1);
				if (ros::param::get("max_min_lp_demo_node/t"+temp_index+"_deg", temp_deg)) {
					temp_node.loc_deg = temp_deg;
					temp_node.type = "target";
					temp_node.id = i+1;

					if (ros::param::get("max_min_lp_demo_node/t"+temp_index+"_neighbor", temp_neighbor) &&
						ros::param::get("max_min_lp_demo_node/t"+temp_index+"_edge_weight", temp_loc_edge_weight)) {
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

			// computeLocalAlgorithm(msg);
		}
		else if (strcmp(m_input_type.c_str(),"text") == 0) {
			for (int i = 0; i < m_num_text_files; i++) {
				ROS_INFO("Output_%d :", i+1);

				// Reinitialization
				// Robot nodes
				int num_robot_node = 0;
				vector<int> robot_id;
				vector<int> robot_loc_deg;
				vector<vector<int> > robot_neighbor;
				vector<vector<float> > robot_loc_edge_weight;
				// Motion primitive to robot nodes
				int num_p_r_node = 0;
				vector<int> p_r_id;
				vector<int> p_r_loc_deg;
				vector<vector<int> > p_r_neighbor;
				vector<vector<float> > p_r_loc_edge_weight;
				// Motion primitive to target nodes
				int num_p_t_node = 0;
				vector<int> p_t_id;
				vector<int> p_t_loc_deg;
				vector<vector<int> > p_t_neighbor;
				vector<vector<float> > p_t_loc_edge_weight;
				// Target nodes
				int num_target_node = 0;
				vector<int> target_id;
				vector<int> target_loc_deg;
				vector<vector<int> > target_neighbor;
				vector<vector<float> > target_loc_edge_weight;

				int text_index = i+1;
				string line;
				string partial_path;
				m_nh.getParam("input_path", partial_path);
				ROS_INFO("    %s", partial_path.c_str());
				string temp_file_path = partial_path+"output_"+boost::lexical_cast<string>(text_index)+".txt";
				ifstream output_file (temp_file_path.c_str());
				if (output_file.is_open())
				{
					getline(output_file,line);
					num_robot_node = boost::lexical_cast<int>(line);
					getline(output_file,line);
					getline(output_file,line);
					num_p_r_node = boost::lexical_cast<int>(line);
					num_p_t_node = boost::lexical_cast<int>(line);
					getline(output_file,line);
					num_target_node = boost::lexical_cast<int>(line);
					getline(output_file,line);
					getline(output_file,line);
					getline(output_file,line);

					if (i == 0) {
						m_outputFile_journal<<num_robot_node<<endl;
						m_outputFile_journal<<num_p_t_node<<endl;
						m_outputFile_journal<<num_target_node<<endl;
					}

					// Build A matrix. (Related with robot and p_r)
					int A[num_robot_node][num_p_r_node];
					for (int j = 0; j < num_robot_node; j++) {
						for (int k = 0; k < num_p_r_node; k++) {
							A[j][k] = 0;
						}
					}

					for (int j = 0; j < num_robot_node; j++) {
						int col_index_text = 0;
						getline(output_file,line);
						for (int k = 0; k < num_p_r_node; k++) {
							if (boost::lexical_cast<int>(line.at(col_index_text)) == 1) {
								A[j][k] = 1;
							}
							col_index_text += 2;
						}
					}

					// for (vector<int>::iterator it = p_r_loc_deg.begin(); it != p_r_loc_deg.end(); ++it) {
					// 	ROS_INFO("p_r_loc_deg = %d", *it);
					// }
					// for (int j = 0; j < num_p_r_node; j++) {
					// 	for (vector<int>::iterator it = p_r_neighbor[j].begin(); it != p_r_neighbor[j].end(); ++it) {
					// 		ROS_INFO("p_r_neighbor = %d", *it);
					// 	}
					// 	for (vector<float>::iterator it = p_r_loc_edge_weight[j].begin(); it != p_r_loc_edge_weight[j].end(); ++it) {
					// 		ROS_INFO("p_r_loc_edge_weight = %.2f", *it);
					// 	}
					// }

					// for (int j = 0; j < num_robot_node; j++) {
					// 	for (int k = 0; k < num_p_r_node; k++) {
					// 		ROS_INFO("A (%d, %d) = %d", j+1, k+1, A[j][k]);
					// 	}
					// }

					getline(output_file,line);

					// Build C matrix. (Related with target and p_t)
					int C[num_target_node][num_p_t_node];
					for (int j = 0; j < num_target_node; j++) {
						for (int k = 0; k < num_p_t_node; k++) {
							C[j][k] = 0;
						}
					}

					for (int j = 0; j < num_target_node; j++) {
						int col_index_text = 0;
						getline(output_file,line);
						for (int k = 0; k < num_p_t_node; k++) {
							if (boost::lexical_cast<int>(line.at(col_index_text)) == 1) {
								C[j][k] = 1;
							}
							col_index_text += 2;
						}
					}

					// Get the node information from A and C matrices.
					// Targets and p_t nodes
					vector<int> find_disconnected_primitive_index;
					for (int j = 0; j < num_target_node; j++) {
						int count_loc_deg = 0;
						vector<int> find_target_neighbor;
						vector<float> find_target_loc_edge_weight;
						for (int k = 0; k < num_p_t_node; k++) {
							if (C[j][k] == 1) {
								count_loc_deg += 1;
								find_target_neighbor.push_back(k+1);
								find_target_loc_edge_weight.push_back(1);
							}
						}

						target_id.push_back(j+1);
						target_loc_deg.push_back(count_loc_deg);
						target_neighbor.push_back(find_target_neighbor);
						target_loc_edge_weight.push_back(find_target_loc_edge_weight);
					}

					int count_p_t_node_after_survival = 0;
					for (int j = 0; j < num_p_t_node; j++) {
						int count_loc_deg = 0;
						vector<int> find_p_t_neighbor;
						vector<float> find_p_t_loc_edge_weight;
						bool check_connected = false;
						for (int k = 0; k < num_target_node; k++) {
							if (C[k][j] == 1) {
								count_loc_deg += 1;
								find_p_t_neighbor.push_back(k+1);
								find_p_t_loc_edge_weight.push_back(1);
								check_connected = true;
							}
						}

						if (check_connected) {
							p_t_id.push_back(j+1);
							p_t_loc_deg.push_back(count_loc_deg);
							p_t_neighbor.push_back(find_p_t_neighbor);
							p_t_loc_edge_weight.push_back(find_p_t_loc_edge_weight);
							count_p_t_node_after_survival += 1;
						}
						else {
							find_disconnected_primitive_index.push_back(j);
						}
					}
					num_p_t_node = count_p_t_node_after_survival;

					// Robots and p_r nodes
					for (int j = 0; j < num_robot_node; j++) {
						int count_loc_deg = 0;
						vector<int> find_robot_neighbor;
						vector<float> find_robot_loc_edge_weight;
						for (int k = 0; k < num_p_r_node; k++) {
							bool check_removed_primitive = false;
							for (vector<int>::iterator it = find_disconnected_primitive_index.begin(); it != find_disconnected_primitive_index.end(); ++it) {
								if (*it == k) {
									check_removed_primitive = true;
								}
							}
							if (check_removed_primitive) {
								continue;
							}

							if (A[j][k] == 1) {
								count_loc_deg += 1;
								find_robot_neighbor.push_back(k+1);
								find_robot_loc_edge_weight.push_back(1);
							}
						}

						robot_id.push_back(j+1);
						robot_loc_deg.push_back(count_loc_deg);
						robot_neighbor.push_back(find_robot_neighbor);
						robot_loc_edge_weight.push_back(find_robot_loc_edge_weight);
					}

					int count_p_r_node_after_survival = 0;
					for (int j = 0; j < num_p_r_node; j++) {
						bool check_removed_primitive = false;
						for (vector<int>::iterator it = find_disconnected_primitive_index.begin(); it != find_disconnected_primitive_index.end(); ++it) {
							if (*it == j) {
								check_removed_primitive = true;
							}
						}
						if (check_removed_primitive) {
							continue;
						}

						int count_loc_deg = 0;
						vector<int> find_p_r_neighbor;
						vector<float> find_p_r_loc_edge_weight;
						for (int k = 0; k < num_robot_node; k++) {
							if (A[k][j] == 1) {
								count_loc_deg += 1;
								find_p_r_neighbor.push_back(k+1);
								find_p_r_loc_edge_weight.push_back(1);
							}
						}

						p_r_id.push_back(j+1);
						p_r_loc_deg.push_back(count_loc_deg);
						p_r_neighbor.push_back(find_p_r_neighbor);
						p_r_loc_edge_weight.push_back(find_p_r_loc_edge_weight);
						count_p_r_node_after_survival += 1;
					}
					num_p_r_node = count_p_r_node_after_survival;

					// Check the number of num_p_r_node matching with the number of num_p_t_node.
					if (num_p_t_node != num_p_r_node) {
						ROS_INFO("ERROR: Something is wrong.");
					}

					// for (vector<int>::iterator it = p_t_loc_deg.begin(); it != p_t_loc_deg.end(); ++it) {
					// 	ROS_INFO("p_t_loc_deg = %d", *it);
					// }
					// for (int j = 0; j < num_p_r_node; j++) {
					// 	for (vector<int>::iterator it = p_t_neighbor[j].begin(); it != p_t_neighbor[j].end(); ++it) {
					// 		ROS_INFO("p_t_neighbor = %d", *it);
					// 	}
					// 	for (vector<float>::iterator it = p_t_loc_edge_weight[j].begin(); it != p_t_loc_edge_weight[j].end(); ++it) {
					// 		ROS_INFO("p_t_loc_edge_weight = %.2f", *it);
					// 	}
					// }

					// for (int j = 0; j < num_target_node; j++) {
					// 	for (int k = 0; k < num_p_t_node; k++) {
					// 		ROS_INFO("C (%d, %d) = %d", j+1, k+1, C[j][k]);
					// 	}
					// }

					output_file.close();
				}
				else {
					ROS_INFO("ERROR: File is not opening.");
				}

				m_gen_r_node.clear();
				m_gen_p_r_node.clear();
				m_gen_p_t_node.clear();
				m_gen_t_node.clear();

				// Add nodes information of the general graph.
				// Robot nodes
				for (int j = 0; j < num_robot_node; j++) {
					max_min_lp_msgs::general_node temp_node;
					temp_node.loc_deg = robot_loc_deg[j];
					temp_node.type = "robot";
					temp_node.id = robot_id[j];

					for (int k = 0; k < robot_loc_deg[j]; k++) {
						temp_node.loc_neighbor.push_back(robot_neighbor[j][k]);
						temp_node.loc_edge_weight.push_back(robot_loc_edge_weight[j][k]);
					}

					m_gen_r_node.push_back(temp_node);
				}
				// Motion primitive to robot nodes
				for (int j = 0; j < num_p_r_node; j++) {
					max_min_lp_msgs::general_node temp_node;
					temp_node.loc_deg = p_r_loc_deg[j];
					temp_node.type = "red";
					temp_node.id = p_r_id[j];

					for (int k = 0; k < p_r_loc_deg[j]; k++) {
						temp_node.loc_neighbor.push_back(p_r_neighbor[j][k]);
						temp_node.loc_edge_weight.push_back(p_r_loc_edge_weight[j][k]);
					}

					m_gen_p_r_node.push_back(temp_node);
				}
				// Motion primitive to target nodes
				for (int j = 0; j < num_p_t_node; j++) {
					max_min_lp_msgs::general_node temp_node;
					temp_node.loc_deg = p_t_loc_deg[j];
					temp_node.type = "blue";
					temp_node.id = p_t_id[j];

					for (int k = 0; k < p_t_loc_deg[j]; k++) {
						temp_node.loc_neighbor.push_back(p_t_neighbor[j][k]);
						temp_node.loc_edge_weight.push_back(p_t_loc_edge_weight[j][k]);
					}

					m_gen_p_t_node.push_back(temp_node);
				}
				// Target nodes
				for (int j = 0; j < num_target_node; j++) {
					max_min_lp_msgs::general_node temp_node;
					temp_node.loc_deg = target_loc_deg[j];
					temp_node.type = "target";
					temp_node.id = target_id[j];

					for (int k = 0; k < target_loc_deg[j]; k++) {
						temp_node.loc_neighbor.push_back(target_neighbor[j][k]);
						temp_node.loc_edge_weight.push_back(target_loc_edge_weight[j][k]);
					}

					m_gen_t_node.push_back(temp_node);
				}

				computeLocalAlgorithm(msg, num_robot_node, num_p_t_node, p_t_id, p_t_loc_deg, p_t_neighbor);
			}
		}
	}
}

void MaxMinLPDemo::computeLocalAlgorithm(const std_msgs::String::ConstPtr& msg, int num_robot_node, int num_p_t_node, vector<int> p_t_id, vector<int> p_t_loc_deg, vector<vector<int> > p_t_neighbor) {
	string temp_robot_state = msg->data.c_str();
	if (strcmp(temp_robot_state.c_str(), "demo") == 0) {
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

		// Get the solution from local algorithm.
		vector<max_min_lp_msgs::general_node> returned_gen_p_r_node = lpc.applyLocalAlgorithm();

		// This part is added for the journal version.
		for (vector<max_min_lp_msgs::general_node>::iterator it = returned_gen_p_r_node.begin(); it != returned_gen_p_r_node.end(); ++it) {
			m_outputFile_journal<<it->z_v<<" ";
		}
		m_outputFile_journal<<endl;

		vector<int> selected_primitive_id;
		for (int i = 0; i < num_robot_node; i++) {
			float first_option = 0;
			float second_option = 0;

			for (vector<max_min_lp_msgs::general_node>::iterator it = returned_gen_p_r_node.begin(); it != returned_gen_p_r_node.end(); ++it) {
				if (it->id == i*2+1) {
					first_option = it->z_v;
				}
				else if (it->id == i*2+2) {
					second_option = it->z_v;
				}
			}

			if (first_option > second_option) {
				selected_primitive_id.push_back(i*2+1);
			}
			else {
				selected_primitive_id.push_back(i*2+2);
			}
		}

		vector<int> targets_observed;
		for (int j = 0; j < num_p_t_node; j++) {
			for (vector<int>::iterator it = selected_primitive_id.begin(); it != selected_primitive_id.end(); ++it) {
				if (p_t_id[j] == *it) {
					for (int k = 0; k < p_t_loc_deg[j]; k++) {
						targets_observed.push_back(p_t_neighbor[j][k]);
					}
				}
			}
		}

		set<int> targets_observed_set(targets_observed.begin(), targets_observed.end());
		targets_observed.clear();
		for (set<int>::iterator it = targets_observed_set.begin(); it != targets_observed_set.end(); ++it) {
			targets_observed.push_back(*it);
		}

		if (m_verbal_flag) {
			ROS_INFO("motion primitives selected:");
		}
		int temp_robot_count = 0;
		for (vector<int>::iterator it = selected_primitive_id.begin(); it != selected_primitive_id.end(); ++it) {
			temp_robot_count += 1;

			if (m_verbal_flag) {
				ROS_INFO("    robot %d's selected motion primitive id = %d", temp_robot_count, *it);
			}
		}
		// ROS_INFO(" ");
		m_count += 1;
		m_outputFile<<m_count<<" "<<(int)targets_observed.size()<<endl;
		ROS_INFO("targets observed: total number = %d", (int)targets_observed.size());
		for (vector<int>::iterator it = targets_observed.begin(); it != targets_observed.end(); ++it) {
			if (m_verbal_flag) {
				ROS_INFO("    target id = %d", *it);
			}
		}
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "max_min_lp_demo");
	MaxMinLPDemo lp;

	ros::spin();
	return 0;
}
