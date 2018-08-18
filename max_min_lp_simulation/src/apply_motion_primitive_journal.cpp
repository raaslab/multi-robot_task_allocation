/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * Important note: in this code a robot is used to take into consideration of the reduction of the step 1 of local algorithm. 
 * Thus, a robot here does not indicate a real robot but an element of a real robot. Instead, ROBOT is used for a real robot.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/27/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/apply_motion_primitive_journal.hpp"

#define PHI 3.141592

apply_motion_primitive::apply_motion_primitive() :
m_num_robot(1), m_num_target(1), m_robot_id(1), m_robot_name(string("robot_0")), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("robot_id", m_robot_id);
	m_private_nh.getParam("robot_name", m_robot_name);

	m_cmd_vel_robot_pub = m_nh.advertise<geometry_msgs::Twist>("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/cmd_vel_mux/input/teleop", 1);
	m_move_service = m_nh.advertiseService("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/move_request", &apply_motion_primitive::move_robot, this);
}

bool apply_motion_primitive::move_robot(max_min_lp_simulation::MoveRobot::Request &req, max_min_lp_simulation::MoveRobot::Response &res) {
	m_check_rotation_direction = req.rotation_direction;
	m_trans_duration = req.trans_duration;
	m_ang_duration = req.ang_duration;
	m_trans_speed = req.trans_speed;
	m_ang_speed = req.ang_speed;

	ROS_INFO("Robot %d trans_duration = %d, ang_duration = %d, trans_speed = %.2f, ang_speed = %.2f", m_robot_id, m_trans_duration, m_ang_duration, m_trans_speed, m_ang_speed);
	ROS_INFO("Robot %d rotation direction = %d", m_robot_id, m_check_rotation_direction);

	double cur_time =ros::Time::now().toSec();
	double prev_time =ros::Time::now().toSec();

	// m_goal_pos.orientation.w
	while (1) {
		cur_time =ros::Time::now().toSec();
		geometry_msgs::Twist cmd_vel_msg;

		if (m_check_rotation_direction == 1) { // CW
			cmd_vel_msg.linear.x = 0;
			cmd_vel_msg.linear.y = 0;
			cmd_vel_msg.linear.z = 0;
			cmd_vel_msg.angular.x = 0;
			cmd_vel_msg.angular.y = 0;
			cmd_vel_msg.angular.z = m_ang_speed;

			m_cmd_vel_robot_pub.publish(cmd_vel_msg);
		}
		else if (m_check_rotation_direction == -1) { // CCW
			cmd_vel_msg.linear.x = 0;
			cmd_vel_msg.linear.y = 0;
			cmd_vel_msg.linear.z = 0;
			cmd_vel_msg.angular.x = 0;
			cmd_vel_msg.angular.y = 0;
			cmd_vel_msg.angular.z = -1*m_ang_speed;

			m_cmd_vel_robot_pub.publish(cmd_vel_msg);
		}
		else { // Remaining stationary
			break;
		}
		if (cur_time-prev_time > m_ang_duration) {
			break;
		}
	}

	cur_time =ros::Time::now().toSec();
	prev_time =ros::Time::now().toSec();

	// m_goal_pos.position.x, m_goal_pos.position.y
	while (1) {
		cur_time =ros::Time::now().toSec();
		geometry_msgs::Twist cmd_vel_msg;

		cmd_vel_msg.linear.x = m_trans_speed;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;

		m_cmd_vel_robot_pub.publish(cmd_vel_msg);

		if (cur_time-prev_time > m_trans_duration) {
			break;
		}
	}

	geometry_msgs::Twist cmd_vel_msg;

	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.linear.y = 0;
	cmd_vel_msg.linear.z = 0;
	cmd_vel_msg.angular.x = 0;
	cmd_vel_msg.angular.y = 0;
	cmd_vel_msg.angular.z = 0;

	m_cmd_vel_robot_pub.publish(cmd_vel_msg);

	m_odom_client = m_nh.serviceClient<max_min_lp_simulation::GetOdom>("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/odom_request", true);
	max_min_lp_simulation::GetOdom srv;
	srv.request.request_odom = string("request");

	if (m_odom_client.call(srv)) {
		geometry_msgs::Pose temp_pos;
		temp_pos = srv.response.return_odom;

		tf::Quaternion q(temp_pos.orientation.x, temp_pos.orientation.y, temp_pos.orientation.z, temp_pos.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		yaw = yaw * 180 / PHI;
		ROS_INFO("Robot %d : (%.2f, %.2f, %.2f)",  m_robot_id, temp_pos.position.x, temp_pos.position.y, yaw);
		res.answer_msg = string("success");

		return true;
	}
	else {
		return false;
	}

	// ROS_INFO(" ");
	// ROS_INFO("(Apply_motion_primitive) ROBOT %d : (%.2f, %.2f)", m_robot_id, temp_pos.position.x, temp_pos.position.y);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "apply_motion_primitive_journal");

	apply_motion_primitive amp;

	ros::spin();
	return 0;
}