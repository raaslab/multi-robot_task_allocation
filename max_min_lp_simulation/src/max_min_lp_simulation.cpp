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

ros::Publisher cmd_vel_robot_1_pub;
ros::Publisher cmd_vel_robot_2_pub;

ros::Publisher cmd_vel_target_1_pub;
ros::Publisher cmd_vel_target_2_pub;
ros::Publisher cmd_vel_target_3_pub;

int count_robot_1 = 0;
int count_robot_2 = 0;

int count_target_1 = 0;
int count_target_2 = 0;
int count_target_3 = 0;

int interval = 1000;

void posRobotCallback_1(const nav_msgs::Odometry::ConstPtr& pos_msg) {
	// Publishers
	geometry_msgs::Twist cmd_vel_msg;

	// Motion primitive 1
	if (count_robot_1 > interval-300 && count_robot_1 < interval) {
		cmd_vel_msg.linear.x = 0.5;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;

		cmd_vel_robot_1_pub.publish(cmd_vel_msg);
	}
	else if (count_robot_1 <= interval-300) {
		cmd_vel_msg.linear.x = 0;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 1.5;

		cmd_vel_robot_1_pub.publish(cmd_vel_msg);
	}
	else {
		cmd_vel_msg.linear.x = 0;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;

		cmd_vel_robot_1_pub.publish(cmd_vel_msg);
	}

	ROS_INFO("Robot 1: linear.x=%f, linear.y=%f, angular.z=%f at count %d", 
		cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z, count_robot_1);

	count_robot_1 += 1;
}

void posRobotCallback_2(const nav_msgs::Odometry::ConstPtr& pos_msg) {
	// Publishers
	geometry_msgs::Twist cmd_vel_msg;

	// Motion primitive 2
	if (count_robot_2 > interval-300 && count_robot_2 < interval) {
		cmd_vel_msg.linear.x = 0.5;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;

		cmd_vel_robot_2_pub.publish(cmd_vel_msg);
	}
	else if (count_robot_2 <= interval-300) {
		cmd_vel_msg.linear.x = 0;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = -1.5;

		cmd_vel_robot_2_pub.publish(cmd_vel_msg);
	}
	else {
		cmd_vel_msg.linear.x = 0;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;

		cmd_vel_robot_2_pub.publish(cmd_vel_msg);
	}

	ROS_INFO("Robot 2: linear.x=%f, linear.y=%f, angular.z=%f at count %d", 
		cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z, count_robot_2);

	count_robot_2 += 1;
}

void posTargetCallback_1(const nav_msgs::Odometry::ConstPtr& pos_msg) {
	// Publishers
	geometry_msgs::Twist cmd_vel_msg;

	if (count_target_1 > interval) {
		cmd_vel_msg.linear.x = 0.1;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;

		cmd_vel_target_1_pub.publish(cmd_vel_msg);
	}

	ROS_INFO("Target 1: linear.x=%f, linear.y=%f, angular.z=%f at count %d", 
		cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z, count_target_1);

	count_target_1 += 1;
}

void posTargetCallback_2(const nav_msgs::Odometry::ConstPtr& pos_msg) {
	// Publishers
	geometry_msgs::Twist cmd_vel_msg;

	if (count_target_2 > interval) {
		cmd_vel_msg.linear.x = 0.1;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;

		cmd_vel_target_2_pub.publish(cmd_vel_msg);
	}
	else {
		cmd_vel_msg.linear.x = 0;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 1.5;

		cmd_vel_target_2_pub.publish(cmd_vel_msg);
	}

	ROS_INFO("Target 2: linear.x=%f, linear.y=%f, angular.z=%f at count %d", 
		cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z, count_target_2);

	count_target_2 += 1;
}

void posTargetCallback_3(const nav_msgs::Odometry::ConstPtr& pos_msg) {
	// Publishers
	geometry_msgs::Twist cmd_vel_msg;

	if (count_target_3 > interval) {
		cmd_vel_msg.linear.x = 0.1;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;

		cmd_vel_target_3_pub.publish(cmd_vel_msg);
	}
	else {
		cmd_vel_msg.linear.x = 0;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 2.5;

		cmd_vel_target_3_pub.publish(cmd_vel_msg);
	}

	count_target_3 += 1;

	ROS_INFO("Target 3: linear.x=%f, linear.y=%f, angular.z=%f at count %d", 
		cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z, count_target_3);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "max_min_lp_simulation");
	ros::NodeHandle nh;

	// Publishers
	// Robots
	cmd_vel_robot_1_pub = nh.advertise<geometry_msgs::Twist>("/turtlebot_1/cmd_vel_mux/input/teleop", 1000);
	cmd_vel_robot_2_pub = nh.advertise<geometry_msgs::Twist>("/turtlebot_2/cmd_vel_mux/input/teleop", 1000);
	// Targets
	cmd_vel_target_1_pub = nh.advertise<geometry_msgs::Twist>("/target_1/cmd_vel_mux/input/teleop", 1000);
	cmd_vel_target_2_pub = nh.advertise<geometry_msgs::Twist>("/target_2/cmd_vel_mux/input/teleop", 1000);
	cmd_vel_target_3_pub = nh.advertise<geometry_msgs::Twist>("/target_3/cmd_vel_mux/input/teleop", 1000);

	// Subscribers
	// Robots
	ros::Subscriber pos_robot_1_sub = nh.subscribe("/turtlebot_1/odom", 1000, posRobotCallback_1);
	ros::Subscriber pos_robot_2_sub = nh.subscribe("/turtlebot_2/odom", 1000, posRobotCallback_2);
	// Targets
	ros::Subscriber pos_target_1_sub = nh.subscribe("/target_1/odom", 1000, posTargetCallback_1);
	ros::Subscriber pos_target_2_sub = nh.subscribe("/target_2/odom", 1000, posTargetCallback_2);
	ros::Subscriber pos_target_3_sub = nh.subscribe("/target_3/odom", 1000, posTargetCallback_3);

	ros::spin();

	return 0;
}