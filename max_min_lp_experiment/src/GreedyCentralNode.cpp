/**
 * Central node for the experiment.
 * Central node only links between a set of motion primitives and a set of targets.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \04/14/2018
 * Copyright 2018. All Rights Reserved.
 */

#include "max_min_lp_experiment/GreedyCentralNode.hpp"

GreedyCentralNode::GreedyCentralNode() :
m_num_robot(1), m_sensing_range(10), m_time_period(5), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("sensing_range", m_sensing_range);
	m_private_nh.getParam("time_period", m_time_period);

	// Services
	m_center_request_service = m_nh.advertiseService("/robot_request", &GreedyCentralNode::requestInitialize, this);
	// Publishers
	m_response_to_robot_pub = m_nh.advertise<std_msgs::String>("/center_response", 1);
	// Subscribers
	m_comm_graph_by_robots_sub = m_nh.subscribe<std_msgs::String>("/robot_after_motion", 10, &GreedyCentralNode::afterAppliedMotionPrimitive, this);
	
	m_request_robot_id = 0;
	m_send_robot_id = 0;
	m_check_request_send = true;

	m_check_finish_action_apply = 0;
	m_time_step = 0;

	targetInitialize();

	ROS_INFO("----------------------------- Time Step %d -----------------------------", m_time_step);
	ROS_INFO(" ");
}

bool GreedyCentralNode::requestInitialize(max_min_lp_experiment::RobotRequest::Request &req, max_min_lp_experiment::RobotRequest::Response &res) {
	if (strcmp(req.state_request.c_str(), "ready") == 0) {
		if (m_request_robot_id == req.robot_id) {
			res.state_answer = "wait";
			m_request_robot_id += 1;
			m_send_robot_id = 0;

			m_robot_id.push_back(req.robot_id);
			m_robot_pose_x.push_back(req.robot_pose.x);
			m_robot_pose_y.push_back(req.robot_pose.y);
		}

		// Once all robots gave their local information to the central node, then do the following.
		if (m_request_robot_id == m_num_robot) {
			if (m_send_robot_id == req.robot_id) { // Here send local information to each robot.
				res.state_answer = "start";

				// Compute observable targets and communicative last robot for each robot.
				vector<int> observed_target_id;

				// Service of request to target_node.
				m_center_request_client = m_nh.serviceClient<max_min_lp_experiment::TargetRequest>("/target_request");
				max_min_lp_experiment::TargetRequest srv;
				srv.request.state_request = "center_ready";

				for (int i = 0; i < m_num_target; i++) {
					double dist_target_robot = sqrt(pow(m_target_pose_x[i]-req.robot_pose.x,2)
						+pow(m_target_pose_y[i]-req.robot_pose.y,2));
					geometry_msgs::Point target_pose;
					target_pose.x = m_target_pose_x[i];
					target_pose.y = m_target_pose_y[i];
					target_pose.z = 0;

					if (dist_target_robot < m_sensing_range) {
						observed_target_id.push_back(m_target_id[i]);
						res.target_id.push_back(m_target_id[i]);
						res.target_pose.push_back(target_pose);
					}
				}

				for (int i = m_send_robot_id-1; i == 0; i--) {
					bool found_comm_robot = false;
					for (vector<int>::iterator it = observed_target_id.begin(); it != observed_target_id.end(); ++it) {
						double dist_target_robot = sqrt(pow(m_target_pose_x[*it]-m_robot_pose_x[i],2)
							+pow(m_target_pose_y[*it]-m_robot_pose_y[i],2));
						if (dist_target_robot < m_sensing_range) {
							res.comm_robot_id = i;
							found_comm_robot = true;
							break;
						}
					}
					if (found_comm_robot) {
						break;
					}
				}

				if (m_send_robot_id+1 == m_num_robot) {
					m_request_robot_id = 0;
				}

				observed_target_id.clear();
				m_send_robot_id += 1;
			}
		}
	}

	return true;
}

void GreedyCentralNode::afterAppliedMotionPrimitive(const std_msgs::String::ConstPtr& msg) {
	if (strcmp(msg->data.c_str(), "action is applied") == 0) {
		m_check_finish_action_apply += 1;

		if (m_check_finish_action_apply == m_num_robot) { // All robots finish applying their actions.
			ROS_INFO("Number of observed targets = %d", (int)m_target_index_used.size());

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

void GreedyCentralNode::targetInitialize() {
	if (ros::param::get("central_node_greedy/num_of_targets", m_num_target)) {}
	else {
		ROS_WARN("Didn't find num_of_targets");
	}
	if (ros::param::get("central_node_greedy/target_id", m_target_id)) {}
	else {
		ROS_WARN("Didn't find target_id");
	}
	if (ros::param::get("central_node_greedy/pose_x", m_target_pose_x)) {}
	else {
		ROS_WARN("Didn't find pose_x");
	}
	if (ros::param::get("central_node_greedy/pose_y", m_target_pose_y)) {}
	else {
		ROS_WARN("Didn't find pose_y");
	}
	if (ros::param::get("central_node_greedy/velocity", m_target_velocity)) {}
	else {
		ROS_WARN("Didn't find velocity");
	}
	if (ros::param::get("central_node_greedy/orientation", m_target_orientation)) {}
	else {
		ROS_WARN("Didn't find orientation");
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "central_node_greedy");
	GreedyCentralNode cn;

	ros::spin();

	return 0;
}
