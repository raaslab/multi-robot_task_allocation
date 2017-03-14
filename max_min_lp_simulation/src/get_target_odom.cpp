/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * Important note: in this code a robot is used to take into consideration of the reduction of the step 1 of local algorithm. 
 * Thus, a robot here does not indicate a real robot but an element of a real robot. Instead, ROBOT is used for a real robot.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/27/2017
 * Copyright 2017. All Rights Reserved.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <boost/lexical_cast.hpp>
#include <max_min_lp_simulation/GetOdom.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

using namespace std;

vector<geometry_msgs::Pose> target_pos;

bool return_odom(max_min_lp_simulation::GetOdom::Request &req, max_min_lp_simulation::GetOdom::Response &res) {
	res.return_target_odom = target_pos;
	return true;
}

void getTargetPos(double t_x, double t_y, double t_z, int t_id) {
	geometry_msgs::Pose temp_pos;
	temp_pos.position.x = t_x;
	temp_pos.position.y = t_y;
	temp_pos.position.z = t_z;

	target_pos[t_id] = temp_pos;

	// ROS_INFO("target %d : (%.2f, %.2f, %.2f)", t_id, t_x, t_y, t_z);
}

void posesStampedCallback(ConstPosesStampedPtr &posesStamped) {
	// cout << posesStamped->DebugString();

	// ::google::protobuf::int32 sec = posesStamped->time().sec();
	// ::google::protobuf::int32 nsec = posesStamped->time().nsec();
	// std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;

	int target_id = 0;
	double temp_x = 0;
	double temp_y = 0;
	double temp_z = 0;

	target_pos.clear();

	// ROS_INFO("posesStampedCallback() is called!!");
	for (int i = 0; i < posesStamped->pose_size(); ++i) {
		const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
		string name = pose.name();
		if (name == std::string("target_1")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_2")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_3")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_4")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_5")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_6")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_7")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_8")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_9")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_10")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_11")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_12")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_13")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_14")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_15")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_16")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_17")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_18")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_19")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_20")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_21")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_22")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_23")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_24")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_25")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_26")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_27")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_28")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_29")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}
		else if (name == std::string("target_30")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			temp_x = position.x();
			temp_y = position.y();
			temp_z = position.z();
		}

		getTargetPos(temp_x, temp_y, temp_z, target_id);

		target_id += 1;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "get_target_odom");
	ros::NodeHandle nh;

	ros::ServiceServer odom_service;

	// Load gazebo
	gazebo::client::setup(argc, argv);

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	// Listen to Gazebo pose info topic
	gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", posesStampedCallback);

	odom_service = nh.advertiseService("/target_odom_request", return_odom);

	ros::spin();
	return 0;
}