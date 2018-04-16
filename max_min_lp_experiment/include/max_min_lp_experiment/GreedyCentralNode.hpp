#ifndef GREEDYCENTRALNODE_HPP_
#define GREEDYCENTRALNODE_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <max_min_lp_experiment/RobotRequest.h>
// #include <max_min_lp_msgs/server_to_robots.h>
// #include <max_min_lp_msgs/server_to_robots_array.h>
// #include <max_min_lp_msgs/target_node.h>
// #include <max_min_lp_msgs/primitive_node.h>
// #include <max_min_lp_simulation/get_odom.hpp>
// #include <max_min_lp_simulation/GetOdom.h>
// #include <max_min_lp_simulation/MotionPrimitiveRequest.h>
// #include <max_min_lp_simulation/GetTotalNumTarget.h>

using namespace std;

class GreedyCentralNode {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Publishers
  ros::Publisher m_response_to_robot_pub;

  // Subscribers
  ros::Subscriber m_comm_graph_by_robots_sub;

  // Services
  ros::ServiceServer m_request_service;

  // Params from the launch file
  int m_num_robot;
  int m_sensing_range;

  int m_send_robot_id;
  int m_request_robot_id;
  bool m_check_request_send;

  // Robot information
  vector<int> m_robot_id;
  vector<float> m_robot_pose_x;
  vector<float> m_robot_pose_y;

  // Target information
  int m_num_target;
  vector<int> m_target_id;
  vector<float> m_target_pose_x;
  vector<float> m_target_pose_y;
  vector<float> m_target_velocity;
  vector<float> m_target_orientation;

  vector<vector<int> > m_primitives_to_targets; // Motion primitives to targets
  vector<vector<int> > m_targets_to_primitives; // Targets to motion primitives

  vector<int> m_target_index_used;

  vector<int> m_optimal_primitive_id;

  int m_check_finish_action_apply;
  int m_time_step;

public:
  GreedyCentralNode(); // Constructor

  bool requestInitialize(max_min_lp_experiment::RobotRequest::Request &req, max_min_lp_experiment::RobotRequest::Response &res);
  void applySequentialLocalAlgorithm(const std_msgs::String::ConstPtr& msg);
  void targetInitialize();
};

#endif