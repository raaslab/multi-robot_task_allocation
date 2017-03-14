/**
 * Generating and testing motion primitives of a robot
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/13/2017
 * Copyright 2016. All Rights Reserved.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <gazebo_msgs/ModelStates.h>
#include <max_min_lp_simulation/get_odom.hpp>
#include <max_min_lp_simulation/GetOdom.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <string>
#include <tf/transform_datatypes.h>

using namespace std;

#define PHI 3.141592

class Robot_Test {
private:
	ros::NodeHandle nh;
	ros::NodeHandle m_private_nh;

	ros::Publisher cmd_vel_robot_pub;
	ros::Publisher m_response_to_robot_pub;

	ros::Subscriber pos_robot_sub;
	ros::Subscriber request_sub;

	ros::ServiceClient m_odom_client;

	int count_robot;
	int interval;

	ros::Timer timer;

	geometry_msgs::Pose odom;

	int m_motion_case_rotation;
	int m_time_interval;

	int m_num_robot;
	int m_robot_id;
	string m_robot_name;
	bool m_verbal_flag;

	int counter;

public:
	Robot_Test() : m_verbal_flag(false), m_num_robot(1), m_robot_id(1), m_robot_name(string("robot_1")), m_private_nh("~") {
		m_private_nh.getParam("verbal_flag", m_verbal_flag);
		m_private_nh.getParam("num_robot", m_num_robot);
		m_private_nh.getParam("robot_id", m_robot_id);
		m_private_nh.getParam("robot_name", m_robot_name);

		count_robot = 0;
		interval = 1000;

		// Publishers
		// Robots
		cmd_vel_robot_pub = nh.advertise<geometry_msgs::Twist>("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/cmd_vel_mux/input/teleop", 1);
		m_response_to_robot_pub = nh.advertise<std_msgs::String>("/robot_status", 1);

		// Subscribers
		// Robots
		pos_robot_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &Robot_Test::posRobotCallback, this);
		request_sub = nh.subscribe("/robot_status", 1, &Robot_Test::applyMotionPrimitives, this);

		m_motion_case_rotation = 9;
		m_time_interval = 15;

		counter = 0;

		// timer = nh.createTimer(ros::Duration(5), &Robot_Test::applyMotionPrimitives, this);
	}
	void posRobotCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
		int size_msg = m_num_robot + 1; // 1 is for 'ground plane' in gazebo.
		int id;

		for (int i = 0; i < size_msg; i++) {
			if (strcmp(msg->name[i].c_str(), m_robot_name.c_str()) == 0) {
				id = i;
			}
		}
		odom = msg->pose[id];
		
		// if (counter > 10) {
		// 	counter = 0;
			if (m_verbal_flag) {
				// ROS_INFO("ROBOT %d : global position (%.2f, %.2f)", m_robot_id, odom.position.x, odom.position.y);
			}
		// }
		// counter += 1;
	}

	void applyMotionPrimitives(const std_msgs::String::ConstPtr& msg) {
		std::string temp_robot_state = msg->data.c_str();
		// if (strcmp(temp_robot_state.c_str(), "x") == 0) {
		// 	geometry_msgs::Twist cmd_vel_msg;

		// 	cmd_vel_msg.linear.x = 0.1;
		// 	cmd_vel_msg.linear.y = 0;
		// 	cmd_vel_msg.linear.z = 0;
		// 	cmd_vel_msg.angular.x = 0;
		// 	cmd_vel_msg.angular.y = 0;
		// 	cmd_vel_msg.angular.z = 0;

		// 	cmd_vel_robot_pub.publish(cmd_vel_msg);

		// 	tf::Quaternion q(odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w);
		// 	tf::Matrix3x3 m(q);
		// 	double roll, pitch, yaw;
		// 	m.getRPY(roll, pitch, yaw);

		// 	yaw = yaw * 180 / PHI;

		// 	ROS_INFO("ROBOT %d (x: %.2f, y = %.2f, theta = %.2f)", m_robot_id, odom.position.x, odom.position.y, yaw);
		// }
		// else if (strcmp(temp_robot_state.c_str(), "z") == 0) {
		// 	geometry_msgs::Twist cmd_vel_msg;

		// 	cmd_vel_msg.linear.x = 0;
		// 	cmd_vel_msg.linear.y = 0;
		// 	cmd_vel_msg.linear.z = 0;
		// 	cmd_vel_msg.angular.x = 0;
		// 	cmd_vel_msg.angular.y = 0;
		// 	cmd_vel_msg.angular.z = -0.1;

		// 	cmd_vel_robot_pub.publish(cmd_vel_msg);

		// 	tf::Quaternion q(odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w);
		// 	tf::Matrix3x3 m(q);
		// 	double roll, pitch, yaw;
		// 	m.getRPY(roll, pitch, yaw);

		// 	yaw = yaw * 180 / PHI;

		// 	ROS_INFO("ROBOT %d (x: %.2f, y = %.2f, theta = %.2f)", m_robot_id, odom.position.x, odom.position.y, yaw);
		// }
		// else if (strcmp(temp_robot_state.c_str(), "test") == 0) {
		string temp_msg = "test_"+boost::lexical_cast<string>(m_robot_id);
		if (strcmp(temp_robot_state.c_str(), temp_msg.c_str()) == 0 || strcmp(temp_robot_state.c_str(), "test") == 0) {
			tf::Quaternion q(odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			yaw = yaw * 180 / PHI;

			// x value: 0.1 m/s = 0.06 m
			// z value: 0.1 = 3.42 (degree)

			double temp_orientation = yaw + 3.42 * m_motion_case_rotation;
			double x_new, y_new;

			if (temp_orientation > 180) {
				temp_orientation -= 360;
			}
			else if (temp_orientation < -180) {
				temp_orientation += 360;
			}
			// This is hard-coded.
			if (temp_orientation >= 0 && temp_orientation < 90) {
				x_new = odom.position.x + 0.06 * (m_time_interval - abs(m_motion_case_rotation)) 
						* cos(temp_orientation * PHI / 180);
				y_new = odom.position.y + 0.06 * (m_time_interval - abs(m_motion_case_rotation)) 
						* sin(temp_orientation * PHI / 180);
			}
			else if (temp_orientation >= 90 && temp_orientation < 180) {
				x_new = odom.position.x - 0.06 * (m_time_interval - abs(m_motion_case_rotation)) 
						* cos((180 - temp_orientation) * PHI / 180);
				y_new = odom.position.y + 0.06 * (m_time_interval - abs(m_motion_case_rotation)) 
						* sin((180 - temp_orientation) * PHI / 180);
			}
			else if (temp_orientation < 0 && temp_orientation >= -90) {
				x_new = odom.position.x + 0.06 * (m_time_interval - abs(m_motion_case_rotation)) 
						* cos((-1) * temp_orientation * PHI / 180);
				y_new = odom.position.y - 0.06 * (m_time_interval - abs(m_motion_case_rotation)) 
						* sin((-1) * temp_orientation * PHI / 180);
			}
			else if (temp_orientation < -90 && temp_orientation >= -180) {
				x_new = odom.position.x - 0.06 * (m_time_interval - abs(m_motion_case_rotation)) 
						* cos((180 + temp_orientation) * PHI / 180);
				y_new = odom.position.y - 0.06 * (m_time_interval - abs(m_motion_case_rotation)) 
						* sin((180 + temp_orientation) * PHI / 180);
			}

			if (m_verbal_flag) {
				ROS_INFO("ROBOT %d current pos (x: %.2f, y = %.2f, theta = %.2f)", m_robot_id, odom.position.x, odom.position.y, yaw);
				// ROS_INFO("ROBOT %d motion primitive = (%.2f, %.2f)", m_robot_id, x_new, y_new);
			}

			for (int i = 0; i < m_time_interval; i++) {
				if (i < m_motion_case_rotation) {
					geometry_msgs::Twist cmd_vel_msg;

					cmd_vel_msg.linear.x = 0;
					cmd_vel_msg.linear.y = 0;
					cmd_vel_msg.linear.z = 0;
					cmd_vel_msg.angular.x = 0;
					cmd_vel_msg.angular.y = 0;
					cmd_vel_msg.angular.z = 0.1;

					cmd_vel_robot_pub.publish(cmd_vel_msg);

					m_odom_client = nh.serviceClient<max_min_lp_simulation::GetOdom>("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/odom_request", true);
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

						ROS_INFO("ROBOT %d : (%.2f, %.2f, %.2f)",  m_robot_id, temp_pos.position.x, temp_pos.position.y, yaw);
					}

					// std::cout<<"at time "<<i+1<<std::endl;
					// ROS_INFO("x: %.2f, y = %.2f, theta = %.2f", odom.position.x, odom.position.y, yaw*180/PHI);

					if (m_verbal_flag) {
						// ROS_INFO("        ROBOT %d : cmd_vel = (%.1f, %.1f, %.1f) (%.1f, %.1f, %.1f)", 
							// m_robot_id, cmd_vel_msg.linear.x, cmd_vel_msg.linear.y,
							// cmd_vel_msg.linear.z, cmd_vel_msg.angular.x, cmd_vel_msg.angular.y, cmd_vel_msg.angular.z);
					}

					usleep(1000000);
				}
				else {
					geometry_msgs::Twist cmd_vel_msg;

					cmd_vel_msg.linear.x = 0.1;
					cmd_vel_msg.linear.y = 0;
					cmd_vel_msg.linear.z = 0;
					cmd_vel_msg.angular.x = 0;
					cmd_vel_msg.angular.y = 0;
					cmd_vel_msg.angular.z = 0;

					cmd_vel_robot_pub.publish(cmd_vel_msg);

					m_odom_client = nh.serviceClient<max_min_lp_simulation::GetOdom>("/robot_"+boost::lexical_cast<string>(m_robot_id)+"/odom_request", true);
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

						ROS_INFO("ROBOT %d : (%.2f, %.2f, %.2f)",  m_robot_id, temp_pos.position.x, temp_pos.position.y, yaw);
					}

					// std::cout<<"at time "<<i+1<<std::endl;
					// ROS_INFO("x: %.2f, y = %.2f, theta = %.2f", odom.position.x, odom.position.y, yaw*180/PHI);

					if (m_verbal_flag) {
						// ROS_INFO("        ROBOT %d : cmd_vel = (%.1f, %.1f, %.1f) (%.1f, %.1f, %.1f)", 
							// m_robot_id, cmd_vel_msg.linear.x, cmd_vel_msg.linear.y,
							// cmd_vel_msg.linear.z, cmd_vel_msg.angular.x, cmd_vel_msg.angular.y, cmd_vel_msg.angular.z);
					}

					usleep(1000000);
				}
			}
		}

		// Publisher
		std_msgs::String _msg;
	    std::stringstream ss;
	    ss<<"test_"+boost::lexical_cast<string>(m_robot_id);
	    _msg.data = ss.str();
	    m_response_to_robot_pub.publish(_msg);
	}

	// void applyMotionPrimitives(const ros::TimerEvent& event) {
	// 	ROS_INFO("applyMotionPrimitives() is called.");
	// 	// Publishers
	// 	geometry_msgs::Twist cmd_vel_msg;

	// 	cmd_vel_msg.linear.x = 0;
	// 	cmd_vel_msg.linear.y = 0;
	// 	cmd_vel_msg.linear.z = 0;
	// 	cmd_vel_msg.angular.x = 0;
	// 	cmd_vel_msg.angular.y = 0;
	// 	cmd_vel_msg.angular.z = 10;

	// 	cmd_vel_robot_pub.publish(cmd_vel_msg);

	// 	tf::Quaternion q(odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w);
	// 	tf::Matrix3x3 m(q);
	// 	double roll, pitch, yaw;
	// 	m.getRPY(roll, pitch, yaw);
	// 	// std::cout << "Roll: " << roll*180/PHI << ", Pitch: " << pitch*180/PHI << ", Yaw: " << yaw*180/PHI << std::endl;

	// 	// ROS_INFO("Robot: linear.x=%f, linear.y=%f, angular.z=%f at count %d", 
	// 	// 	cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z, count_robot);

	// 	count_robot += 1;
	// }
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_test");

	Robot_Test rt1;

	ros::spin();

	return 0;
}