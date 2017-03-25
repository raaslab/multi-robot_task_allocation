/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/26/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPGreedyCentralNode.hpp"

MaxMinLPGreedyCentralNode::MaxMinLPGreedyCentralNode() :
m_num_robot(1), m_num_target(1), m_fov(10), m_num_motion_primitive(10), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("fov", m_fov);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);

	// // Services
	m_service = m_nh.advertiseService("/robot_request", &MaxMinLPGreedyCentralNode::initialize, this);
	m_primitive_service = m_nh.advertiseService("/motion_primitive_request", &MaxMinLPGreedyCentralNode::sendMotionPrimitive, this);
	m_total_num_target_service = m_nh.advertiseService("/total_num_target_request", &MaxMinLPGreedyCentralNode::get_total_num_target, this);

	m_request_robot_id = 1;
	m_send_robot_id = 1;
	m_check_request_send = true;

	m_response_to_robot_pub = m_nh.advertise<std_msgs::String>("/robot_status", 1);

	m_outputFile.open("/home/yoon/yoon/max_min_ws/src/max_min_lp_simulation/data/greedy/results_for_debug.txt");

	// // Subscribers
	m_comm_graph_by_robots_sub = m_nh.subscribe<std_msgs::String>("/robot_comm_graph", 1000, &MaxMinLPGreedyCentralNode::applySequentialLocalAlgorithm, this);

	geometry_msgs::Pose dummy_target_pos;
	for (int i = 0; i < m_num_target; i++) {
		m_temp_target_name.push_back("target_"+boost::lexical_cast<string>(i+1));
		m_temp_target_pos.push_back(dummy_target_pos);
	}

	// m_target_1_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPGreedyCentralNode::updatePose, this, _1, 0));
	// m_target_2_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPGreedyCentralNode::updatePose, this, _1, 1));
	// m_target_3_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPGreedyCentralNode::updatePose, this, _1, 2));

	m_check_finish_action_apply = 0;
	m_total_num_target_covered = 0;
	m_time_step = 0;

	ROS_INFO("----------------------------- Time Step %d -----------------------------", m_time_step);
	ROS_INFO(" ");
}

bool MaxMinLPGreedyCentralNode::initialize(max_min_lp_simulation::MessageRequest::Request &req, max_min_lp_simulation::MessageRequest::Response &res) {
	if (strcmp(req.state_check.c_str(), "ready") == 0) {
		if (m_request_robot_id == req.robot_id) {
			m_robot_info.robot_status.push_back(boost::lexical_cast<string>("initialized"));
			
			max_min_lp_msgs::server_to_robots temp_server_to_robots;
			temp_server_to_robots.robot_id = req.robot_id;

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
			
			res.state_answer = "wait";
			m_request_robot_id += 1;
			m_send_robot_id = 1;
		}

		// Once all robots gave their local information to the central node, then do the following.
		if (m_request_robot_id == (m_num_robot+1)) {
			if (m_check_request_send) { // Match relavent motion primitives with targets on the basis of FoV. This happens only once in the initialization step.
				m_primitive_id.clear();
				m_primitive_x_pos.clear();
				m_primitive_y_pos.clear();
				m_temp_target_pos.clear();
				m_target_id.clear();
				m_target_x_pos.clear();
				m_target_y_pos.clear();
				m_primitives_to_targets.clear();
				m_targets_to_primitives.clear();
				m_optimal_primitive_id.clear();

				m_target_odom_client = m_nh.serviceClient<max_min_lp_simulation::GetOdom>("/target_odom_request");
				max_min_lp_simulation::GetOdom srv;
				srv.request.request_odom = string("request");

				if (m_target_odom_client.call(srv)) {
					m_temp_target_pos = srv.response.return_target_odom;
				}
				else {
					ROS_INFO("ERROR: Targets are not property obtained.");
				}

				// Obtain target information.
				for (int i = 0; i < m_num_target; i++) {
					m_target_id.push_back(i+1);
					m_target_x_pos.push_back(m_temp_target_pos[i].position.x);
					m_target_y_pos.push_back(m_temp_target_pos[i].position.y);
				}

				// Obtain neighbor primitive information.
				// Firstly, see all motion primitives to pair each with targets that are in the FoV.
				int count_num_total_primitives = 0;
				for (vector<max_min_lp_msgs::server_to_robots>::iterator it = m_robot_info.each_robot.begin(); it != m_robot_info.each_robot.end(); ++it) {
					for (int i = 0; i < it->primitive_id.size(); i++) {
						m_primitive_id.push_back(count_num_total_primitives+1);
						m_primitive_x_pos.push_back(it->p_x_pos[i]);
						m_primitive_y_pos.push_back(it->p_y_pos[i]);
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

				// Then this time see all targets to pair each with motion primitives that are in the FoV.
				for (int i = 0; i < m_target_id.size(); i++) {
					vector<int> temp_targets_to_primitives;
					for (int j = 0; j < m_primitive_id.size(); j++) {
						float dist_primitive_to_target = sqrt(pow((m_target_x_pos[i] - m_primitive_x_pos[j]), 2) + pow((m_target_y_pos[i] - m_primitive_y_pos[j]), 2));
						
						if (dist_primitive_to_target <= m_fov) {
							temp_targets_to_primitives.push_back(m_primitive_id[j]);
						}
					}
					m_targets_to_primitives.push_back(temp_targets_to_primitives);
				}

				// Get ready to send local information to each corresponding robot using the above relationships between motion primitives and targets.
				// for (vector<max_min_lp_msgs::server_to_robots>::iterator it = m_robot_info.each_robot.begin(); it != m_robot_info.each_robot.end(); ++it) {
				// 	for(int i = 0; i < it->primitive_id.size(); i++) {
				// 		if (m_primitives_to_targets[i].size() == 0) { // No targets are connected to this motion primitive.
				// 			it->target_exist.push_back(false);
				// 		}
				// 		else { // Targets are connected to this motion primitive.
				// 			it->target_exist.push_back(true);
				// 			max_min_lp_msgs::target_node temp_target_node;
							
				// 			for (vector<int>::iterator itt = m_primitives_to_targets[i].begin(); itt != m_primitives_to_targets[i].end(); ++itt) {
				// 				temp_target_node.target_id.push_back(*itt);
				// 				temp_target_node.t_x_pos.push_back(m_target_x_pos[*itt - 1]);
				// 				temp_target_node.t_y_pos.push_back(m_target_y_pos[*itt - 1]);

				// 				max_min_lp_msgs::primitive_node temp_primitive_node;

				// 				if (m_targets_to_primitives[*itt - 1].size() == 0) { // No neighbor motion primitives are connected to this target.
				// 					temp_target_node.neighbor_primitive_exist.push_back(false);
				// 				}
				// 				else {
				// 					temp_target_node.neighbor_primitive_exist.push_back(true);

				// 					for (vector<int>::iterator ittt = m_targets_to_primitives[*itt - 1].begin(); ittt != m_targets_to_primitives[*itt - 1].end(); ++ittt) {
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

				// Decide a motion primitive for each robot while considering collision avoidance.
				// vector<geometry_msgs::Pose> object_pos_for_collision;
				// for (int i = 0; i < m_num_target; i++) {
				// 	geometry_msgs::Pose temp_target_pos;
				// 	temp_target_pos.position.x = m_temp_target_pos[i].position.x;
				// 	temp_target_pos.position.y = m_temp_target_pos[i].position.y;
				// 	object_pos_for_collision.push_back(temp_target_pos);
				// }

				m_target_index_used.clear();
				int temp_robot_count = 0;

				for (int i = 0; i < m_num_target; i++) {
					ROS_INFO("**** target %d : (%.2f, %.2f)", i+1, m_temp_target_pos[i].position.x, m_temp_target_pos[i].position.y);
				}

				for (vector<max_min_lp_msgs::server_to_robots>::iterator it = m_robot_info.each_robot.begin(); it != m_robot_info.each_robot.end(); ++it) {
					vector<int> num_targets_detected_by_primitive;
					vector<float> sum_distance_to_targets;
					bool target_observed = false;

					temp_robot_count += 1;

					for (int i = 0; i < it->primitive_id.size(); i++) {
						ROS_INFO("**** ROBOT %d primitive %d : (%.2f, %.2f)", temp_robot_count, i+1, it->p_x_pos[i], it->p_y_pos[i]);
					}

					vector<vector<int> > temp_target_index_used;
					for (int i = 0; i < it->primitive_id.size(); i++) {
						float dist_sum = 0;
						vector<int> temp_target_index_used_at_primitive;
						// Sum all distances to targets.
						for (int j = 0; j < m_num_target; j++) {
							bool check_target_used = false;
							for (vector<int>::iterator itt = m_target_index_used.begin(); itt != m_target_index_used.end(); ++itt) {
								if (j == *itt) {
									check_target_used = true;
								}
							}

							if (check_target_used) {
								continue;
							}

							float dist_primitive_to_target = sqrt(pow((m_temp_target_pos[j].position.x -it->p_x_pos[i]), 2) 
															+ pow((m_temp_target_pos[j].position.y - it->p_y_pos[i]), 2));
							if (dist_primitive_to_target <= m_fov && dist_primitive_to_target > 0.3) {
								dist_sum += dist_primitive_to_target;
								temp_target_index_used_at_primitive.push_back(j);

								ROS_INFO("**** ROBOT %d : primitive %d : observed target = %d", temp_robot_count, i+1, j+1);
							}
						}

						if (dist_sum != 0) {
							target_observed = true;
						}

						sum_distance_to_targets.push_back(dist_sum);

						// Check the collision avoidance.
						// num_targets_detected_by_primitive.push_back(m_primitives_to_targets[i].size());
						// for (int j = 0; j < object_pos_for_collision.size(); j++) {
						// 	float dist_primitive_to_object = sqrt(pow((object_pos_for_collision[j].position.x - m_primitive_x_pos[i]), 2) 
						// 									+ pow((object_pos_for_collision[j].position.y - m_primitive_y_pos[i]), 2));
						// 	if (dist_primitive_to_object < 0.3) { // If any objects are within 2 m of the corresponding motion primitive, this motion primitive is discarded.  (Hard-coded.)
						// 		num_targets_detected_by_primitive[i] = 0;
						// 	}
						// }

						temp_target_index_used.push_back(temp_target_index_used_at_primitive);
					}

					if (!target_observed) { // No targets are observed by this robot. Therefore, take a random motion primitive. At this moment, just move forward. (This should be changed later)

						m_optimal_primitive_id.push_back(2); // Remain staionary.

						// Include the selected motion primitive in a set of objects for the collision avoidance.
						// geometry_msgs::Pose temp_target_pos;
						// temp_target_pos.position.x = m_primitive_x_pos[2];
						// temp_target_pos.position.y = m_primitive_y_pos[2];
						// object_pos_for_collision.push_back(temp_target_pos);

						continue;
					}

					// Decide a motion primitive for the control action by finding the one with the largest number of targets that can be observed.
					// vector<int>::iterator temp_max_iterator = max_element(num_targets_detected_by_primitive.begin(), num_targets_detected_by_primitive.end());
					// vector<int> temp_max_index;
					// int index_count = 0;
					// for (vector<int>::iterator itt = num_targets_detected_by_primitive.begin(); itt != num_targets_detected_by_primitive.end(); ++itt) {
					// 	if (*itt == *temp_max_iterator) {
					// 		temp_max_index.push_back(index_count);
					// 	}

					// 	index_count += 1;
					// }

					int max_number = 0;
					int temp_max_index;
					for (int i = 0; i < it->primitive_id.size(); i++) {
						if (temp_target_index_used[i].size() > max_number) {
							max_number = temp_target_index_used[i].size();
							temp_max_index = i;
						}
					}

					// Hard-coded. It only works for two motion primitives
					if (temp_target_index_used[0].size() == temp_target_index_used[1].size()) {
						temp_max_index = 1;
					}

					m_optimal_primitive_id.push_back(temp_max_index+1);

					for (vector<int>::iterator itt = temp_target_index_used[temp_max_index].begin(); itt != temp_target_index_used[temp_max_index].end(); ++itt) {
						m_target_index_used.push_back(*itt);
					}

					// if (temp_max_index.size() > 1) { // Compare the distance
						// int find_id = 0;
						// float min_value = 10000;
						// for (vector<int>::iterator itt = temp_max_index.begin(); itt != temp_max_index.end(); ++itt) { // Find the mimimum.
						// 	if (sum_distance_to_targets[*itt] < min_value) {
						// 		min_value = sum_distance_to_targets[*itt];
						// 		find_id = *itt;
						// 	}
						// }

						// selected_primitive_id = find_id + 1; // Primitive id

					// 	m_optimal_primitive_id.push_back(2);
					// }
					// else {
						// selected_primitive_id = temp_max_index[0]+1; // Primitive id
					// 	m_optimal_primitive_id.push_back(temp_max_index[0]+1);
					// }

					// Include the selected motion primitive in a set of objects for the collision avoidance.
					// geometry_msgs::Pose temp_target_pos;
					// temp_target_pos.position.x = m_primitive_x_pos[selected_primitive_id-1];
					// temp_target_pos.position.y = m_primitive_y_pos[selected_primitive_id-1];
					// object_pos_for_collision.push_back(temp_target_pos);
				}

				for (int i = 0; i < m_optimal_primitive_id.size(); i++) {
					ROS_INFO("********** ROBOT %d : selected primitive id = %d", i+1, m_optimal_primitive_id[i]);
				}

				m_check_request_send = false;
			} // if (m_check_request_send)

			if (m_send_robot_id == req.robot_id) { // Here send local information to each robot.
				res.state_answer = "start";
				m_send_robot_id += 1;

				for (vector<max_min_lp_msgs::server_to_robots>::iterator it = m_robot_info.each_robot.begin(); it != m_robot_info.each_robot.end(); ++it) {
					if (it->robot_id == req.robot_id) {
						res.neighbor_info = *it; // Send the corresponding local info to the robot requested.
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

void MaxMinLPGreedyCentralNode::applySequentialLocalAlgorithm(const std_msgs::String::ConstPtr& msg) {
	if (strcmp(msg->data.c_str(), "action is applied") == 0) {
		m_check_finish_action_apply += 1;

		if (m_check_finish_action_apply == m_num_robot) { // All robots finish applying their actions.
			// Writing the results to the text files.
			// int text_index = i+1;
			// string line;
			// string temp_file_path = "/home/yoon/yoon/max_min_ws/src/max_min_lp_demo/data/output_"+boost::lexical_cast<string>(text_index)+".txt";
			// ifstream output_file (temp_file_path.c_str());

			// for (int i = 0; i < m_target_id.size(); i++) {
			// 	float temp_target_x_pos = m_target_x_pos[i];
			// 	float temp_target_y_pos = m_target_y_pos[i];

			// 	if (temp_target_x_pos < 0.001 && temp_target_x_pos > 0) {
			// 		temp_target_x_pos = 0;
			// 	}
			// 	if (temp_target_y_pos < 0.001 && temp_target_y_pos > 0) {
			// 		temp_target_y_pos = 0;
			// 	}

			// 	m_target_outputFile<<setprecision(2)<<temp_target_x_pos<<" "<<setprecision(2)<<temp_target_y_pos<<" ";
			// }
			// m_target_outputFile<<endl;

			// Find all targets observed by any ROBOTs. (Current result)
			m_total_num_target_covered = (int)m_target_index_used.size();
			ROS_INFO("Number of observed targets = %d", (int)m_target_index_used.size());

			m_outputFile<<setprecision(2)<<ros::Time::now()<<" "<<(int)m_target_index_used.size()<<endl;

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
			m_check_finish_action_apply = 0;

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

bool MaxMinLPGreedyCentralNode::get_total_num_target(max_min_lp_simulation::GetTotalNumTarget::Request &req, max_min_lp_simulation::GetTotalNumTarget::Response &res) {
	res.total_num_target = m_total_num_target_covered;
	return true;
}

void MaxMinLPGreedyCentralNode::updatePose(const gazebo_msgs::ModelStates::ConstPtr& msg, int target_id) {
	int size_msg = m_num_robot + m_num_target + 1; // 1 is for 'ground plane' in gazebo.
	int id;
	for (int i = 0; i < size_msg; i++) {
		if (strcmp(msg->name[i].c_str(), m_temp_target_name[target_id].c_str()) == 0) {
			id = i;
		}
	}

	m_temp_target_pos[target_id] = msg->pose[id];
}

bool MaxMinLPGreedyCentralNode::sendMotionPrimitive(max_min_lp_simulation::MotionPrimitiveRequest::Request &req, max_min_lp_simulation::MotionPrimitiveRequest::Response &res) {
	res.selected_primitive_id = m_optimal_primitive_id[req.request_ROBOT_id-1];
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_central_node_greedy");

	MaxMinLPGreedyCentralNode cn;

	ros::spin();

	return 0;
}