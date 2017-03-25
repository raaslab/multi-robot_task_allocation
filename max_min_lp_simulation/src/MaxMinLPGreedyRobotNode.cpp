/**
 * Each robot node for the simulation
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/13/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/MaxMinLPGreedyRobotNode.hpp"

#define PHI 3.141592

MaxMinLPGreedyRobotNode::MaxMinLPGreedyRobotNode() :
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

	m_response_to_server_pub = m_nh.advertise<std_msgs::String>("/robot_comm_graph", 1);

	// Output files
	string line;
	string temp_file_path = "/home/yoon/yoon/max_min_ws/src/max_min_lp_simulation/data/greedy/robots_"+boost::lexical_cast<string>(m_robot_id)+".txt";
	m_robot_outputFile.open(temp_file_path.c_str());

	// Subscribers
	m_request_sub = m_nh.subscribe("/robot_status", 1000, &MaxMinLPGreedyRobotNode::applyMotionPrimitives, this);
	m_odom_sub = m_nh.subscribe("/gazebo/model_states", 1000, &MaxMinLPGreedyRobotNode::updateOdom, this);

	count_computeMotionPrimitives = 0;
	m_count_initialize_func = 0;

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

void MaxMinLPGreedyRobotNode::updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg) {
	int size_msg = m_num_robot + m_num_target + 1; // 1 is for 'ground plane' in gazebo.
	int id;
	for (int i = 0; i < size_msg; i++) {
		if (strcmp(msg->name[i].c_str(), m_robot_name.c_str()) == 0) {
			id = i;
		}
	}
	m_pos = msg->pose[id];
}

void MaxMinLPGreedyRobotNode::applyMotionPrimitives(const std_msgs::String::ConstPtr& msg) {
	// Write results to output files.
	float temp_robot_x_pos = m_pos.position.x;
	float temp_robot_y_pos = m_pos.position.y;

	if (temp_robot_x_pos < 0.001 && temp_robot_x_pos > 0) {
		temp_robot_x_pos = 0;
	}
	if (temp_robot_y_pos < 0.001 && temp_robot_y_pos > 0) {
		temp_robot_y_pos = 0;
	}

	m_robot_outputFile<<setprecision(2)<<temp_robot_x_pos<<" "<<setprecision(2)<<temp_robot_y_pos<<" ";
	m_robot_outputFile<<endl;

	bool result_success = initialize();

	if (result_success) {
		ROS_INFO("The %s is initiated.", m_robot_name.c_str());

		m_primitive_client = m_nh.serviceClient<max_min_lp_simulation::MotionPrimitiveRequest>("/motion_primitive_request");
		max_min_lp_simulation::MotionPrimitiveRequest srv;
		srv.request.request_ROBOT_id = m_robot_id;

		if (m_primitive_client.call(srv)) {
			if (m_verbal_flag) {
				ROS_INFO("ROBOT %d is in the getMotionPrimitive() step", m_robot_id);
			}

			m_selected_primitive_id = srv.response.selected_primitive_id;

			// if (m_verbal_flag) {
				ROS_INFO("ROBOT %d accepted a service call (in the getMotionPrimitive() step)", m_robot_id);
				ROS_INFO("ROBOT %d : selected primitive id = %d", m_robot_id, m_selected_primitive_id);
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

bool MaxMinLPGreedyRobotNode::initialize() {
	m_client = m_nh.serviceClient<max_min_lp_simulation::MessageRequest>("/robot_request");
	max_min_lp_simulation::MessageRequest srv;
	srv.request.robot_id = m_robot_id;
	srv.request.state_check = "ready";

	// Compute motion primivites.
	if (m_count_initialize_func == 0) {
	  	m_motion_primitive_pose.clear();
		m_motion_primitive_pose = computeMotionPrimitives();
	}
	srv.request.motion_primitive_info = m_motion_primitive_pose;

	if (m_client.call(srv)) {
		if (strcmp(srv.response.state_answer.c_str(), "start") == 0) {
			// m_local_info
			m_count_initialize_func = 0;
			return true;
		}
		else {
			m_count_initialize_func += 1;
			return MaxMinLPGreedyRobotNode::initialize();
		}
	}
	else {
		ROS_INFO("Fail to communicate with the server.");
		return false;
	}
}

vector<geometry_msgs::Pose> MaxMinLPGreedyRobotNode::computeMotionPrimitives() {
	// At this moment, tweak this part. Just consider the case when the number of motion primitives considered is five. This should be changed later.
	// Also, this motion primitives are based on the 1m/s for x-direction and 1rad/s for z-direction.
	vector<geometry_msgs::Pose> temp_motion_primitive;

	if (m_verbal_flag) {
		ROS_INFO("ROBOT %d is in the computeMotionPrimitives() step", m_robot_id);
	}

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
	m_motion_case_rotation.push_back(random_case);

	tf::Quaternion q(m_pos.orientation.x, m_pos.orientation.y, m_pos.orientation.z, m_pos.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	yaw = yaw * 180 / PHI;

	double x_new, y_new;
	double temp_orientation;

	m_check_rotation_direction.clear();

	for (int i = 0; i < m_num_motion_primitive; i++) {
		if (i == 0) {
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
		else if(i == 1) {
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
	ros::init(argc, argv, "max_min_lp_robot_node_greedy");

	MaxMinLPGreedyRobotNode rn;

	ros::spin();
	return 0;
}