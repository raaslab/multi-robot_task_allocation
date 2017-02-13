/**
 * Simulate turtlebot robots with moving objects
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \12/09/2016
 * Copyright 2016. All Rights Reserved.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <tf/transform_datatypes.h>

#define phi 3.141592

class Robot_Test {
private:
	ros::NodeHandle nh;

	ros::Publisher cmd_vel_robot_pub;
	ros::Subscriber request_sub;

	int count_robot;
	int interval;

	ros::Subscriber pos_robot_sub;
	ros::Timer timer;

	nav_msgs::Odometry odom;

public:
	Robot_Test() {
		count_robot = 0;
		interval = 1000;

		// Publishers
		// Robots
		cmd_vel_robot_pub = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel_mux/input/teleop", 1000);
		request_sub = nh.subscribe("/robot_status", 1000, &Robot_Test::applyMotionPrimitives, this);

		// Subscribers
		// Robots
		pos_robot_sub = nh.subscribe<nav_msgs::Odometry>("/robot_1/odom", 1000, &Robot_Test::posRobotCallback_1, this);

		// timer = nh.createTimer(ros::Duration(5), &Robot_Test::applyMotionPrimitives, this);
	}
	void posRobotCallback_1(const nav_msgs::Odometry::ConstPtr& pos_msg) {
		odom = *pos_msg;
	}

	void applyMotionPrimitives(const std_msgs::String::ConstPtr& msg) {
		std::string temp_robot_state = msg->data.c_str();
		if (strcmp(temp_robot_state.c_str(), "x") == 0) {
			geometry_msgs::Twist cmd_vel_msg;

			cmd_vel_msg.linear.x = 1;
			cmd_vel_msg.linear.y = 0;
			cmd_vel_msg.linear.z = 0;
			cmd_vel_msg.angular.x = 0;
			cmd_vel_msg.angular.y = 0;
			cmd_vel_msg.angular.z = 0;

			cmd_vel_robot_pub.publish(cmd_vel_msg);

			tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			std::cout << "Roll: " << roll*180/phi << ", Pitch: " << pitch*180/phi << ", Yaw: " << yaw*180/phi << std::endl;
		}
		else if (strcmp(temp_robot_state.c_str(), "z") == 0) {
			geometry_msgs::Twist cmd_vel_msg;

			cmd_vel_msg.linear.x = 0;
			cmd_vel_msg.linear.y = 0;
			cmd_vel_msg.linear.z = 0;
			cmd_vel_msg.angular.x = 0;
			cmd_vel_msg.angular.y = 0;
			cmd_vel_msg.angular.z = 1;

			cmd_vel_robot_pub.publish(cmd_vel_msg);

			tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			std::cout << "Roll: " << roll*180/phi << ", Pitch: " << pitch*180/phi << ", Yaw: " << yaw*180/phi << std::endl;
		}
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

	// 	tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	// 	tf::Matrix3x3 m(q);
	// 	double roll, pitch, yaw;
	// 	m.getRPY(roll, pitch, yaw);
	// 	// std::cout << "Roll: " << roll*180/phi << ", Pitch: " << pitch*180/phi << ", Yaw: " << yaw*180/phi << std::endl;

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