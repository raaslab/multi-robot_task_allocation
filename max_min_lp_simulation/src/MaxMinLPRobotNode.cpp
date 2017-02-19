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

	m_num_constraints = 1; // When the |V_i| <= 2, the reduction is not required. Therefore, the number of constraints is one.
	m_constraint_value = 1;

	request_sub = m_nh.subscribe("/robot_status", 1000, &MaxMinLPRobotNode::applyMotionPrimitives, this);

	// Publishers
	// m_request_pub = m_nh.advertise<std_msgs::String>("/max_min_lp_robot_node/"+m_robot_name+"/request", 1);

	// Subscribers
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
		ROS_INFO("The %s is initiated.", m_robot_name.c_str());
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

	// Apply the reduction when |V_i| is greater than 2.
	if (m_num_motion_primitive > 2) {
		m_num_constraints = m_num_motion_primitive * (m_num_motion_primitive - 1) / 2;
		srv.request.num_constraints = m_num_constraints;
		m_constraint_value = float(1) / m_num_motion_primitive;
		srv.request.constraint_value = m_constraint_value;
	}

	if (m_client.call(srv)) {
		if (strcmp(srv.response.state_answer.c_str(), "start") == 0) {
			m_max_neighbor = srv.response.max_neighbor;
			m_gen_r_node = srv.response.gen_r_node;
			m_gen_p_r_node = srv.response.gen_p_r_node;
			m_gen_p_t_node = srv.response.gen_p_t_node;
			m_gen_t_node = srv.response.gen_t_node;

			return true;
		}
		else {
			return MaxMinLPRobotNode::initialize();
		}
	}
	else {
		ROS_INFO("Fail to communicate with the server.");
		return false;
	}
}

vector<geometry_msgs::Pose> MaxMinLPRobotNode::computeMotionPrimitives() {
	// At this moment, tweak this part. Just consider the case when the number of motion primitives considered is five. This should be changed later.
	// Also, this motion primitives are based on the 1m/s for x-direction and 1rad/s for z-direction.
	vector<geometry_msgs::Pose> temp_motion_primitive;

	// This is hard-coded.
	int m_motion_case_rotation[m_num_motion_primitive] = {4, 2, 0, -2, -4};

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

		ROS_INFO("%s: %d'th motion primitiv = (%f, %f)", m_robot_name.c_str(), i+1, x_new, y_new);

		temp_motion_primitive.push_back(temp_motion_primitive_instance);
	}

	return temp_motion_primitive;
}

void MaxMinLPRobotNode::updateGraph() {
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_robot_node");

	MaxMinLPRobotNode rn;

	ros::spin();
	return 0;
}