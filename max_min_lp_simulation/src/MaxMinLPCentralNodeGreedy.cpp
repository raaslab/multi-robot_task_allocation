/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/15/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPCentralNodeGreedy.hpp"

MaxMinLPCentralNode::MaxMinLPCentralNode() :
m_num_robot(1), m_num_target(1), m_fov(10), m_num_motion_primitive(10), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("fov", m_fov);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);

	// // Services
	m_service = m_nh.advertiseService("/robot_request", &MaxMinLPCentralNode::initialize, this);

	m_request_robot_id = 1;
	m_send_robot_id = 1;
	m_check_request_send = true;

	// // Subscribers
	geometry_msgs::Pose dummy_target_pos;
	for (int i = 0; i < m_num_target; i++) {
		m_temp_target_name.push_back("target_"+boost::lexical_cast<string>(i+1));
		m_temp_target_pos.push_back(dummy_target_pos);
	}
	m_target_1_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNode::updatePose, this, _1, 0));
	m_target_2_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNode::updatePose, this, _1, 1));
	m_target_3_sub = m_nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(&MaxMinLPCentralNode::updatePose, this, _1, 2));
}

bool MaxMinLPCentralNode::initialize(max_min_lp_simulation::MessageRequest::Request &req, max_min_lp_simulation::MessageRequest::Response &res) {
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

			m_robot_info.each_robot.push_back(temp_server_to_robots);
			
			res.state_answer = "wait";
			m_request_robot_id += 1;
			m_send_robot_id = 1;
		}

		// Once all robots gave their local information to the central node, then do the following.
		if (m_request_robot_id == (m_num_robot+1)) {
			if (m_check_request_send) { // Match relavent motion primitives with targets on the basis of FoV. This happens only once in the initialization step.
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
				for (vector<max_min_lp_msgs::server_to_robots>::iterator it = m_robot_info.each_robot.begin(); it != m_robot_info.each_robot.end(); ++it) {
					for(int i = 0; i < it->primitive_id.size(); i++) {
						if (m_primitives_to_targets[i].size() == 0) { // No targets are connected to this motion primitive.
							it->target_exist.push_back(false);
						}
						else { // Targets are connected to this motion primitive.
							it->target_exist.push_back(true);
							max_min_lp_msgs::target_node temp_target_node;
							
							for (vector<int>::iterator itt = m_primitives_to_targets[i].begin(); itt != m_primitives_to_targets[i].end(); ++itt) {
								temp_target_node.target_id.push_back(*itt);
								temp_target_node.t_x_pos.push_back(m_target_x_pos[*itt - 1]);
								temp_target_node.t_y_pos.push_back(m_target_y_pos[*itt - 1]);

								max_min_lp_msgs::primitive_node temp_primitive_node;

								if (m_targets_to_primitives[*itt - 1].size() == 0) { // No neighbor motion primitives are connected to this target.
									temp_target_node.neighbor_primitive_exist.push_back(false);
								}
								else {
									temp_target_node.neighbor_primitive_exist.push_back(true);

									for (vector<int>::iterator ittt = m_targets_to_primitives[*itt - 1].begin(); ittt != m_targets_to_primitives[*itt - 1].end(); ++ittt) {
										if (*ittt == it->primitive_id[i]) { // Array of targets to primitives include all possible primitives so the primitive we are looking at now must not be taken into consideration.
											continue;
										}
										temp_primitive_node.neighbor_primitive_id.push_back(*ittt);
										temp_primitive_node.neighbor_p_x_pos.push_back(m_primitive_x_pos[*ittt - 1]);
										temp_primitive_node.neighbor_p_y_pos.push_back(m_primitive_y_pos[*ittt - 1]);
									}
								}

								temp_target_node.connect_primitive.push_back(temp_primitive_node);
							}

							it->connect_target.push_back(temp_target_node);
						}
					}
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