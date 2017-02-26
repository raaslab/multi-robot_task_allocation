#ifndef MaxMinLPGREEDYRobotNode_HPP_
#define MaxMinLPGREEDYRobotNode_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <tf/transform_datatypes.h>
#include <max_min_lp_msgs/general_node.h>
#include <max_min_lp_msgs/general_node_array.h>
#include <max_min_lp_msgs/layered_node.h>
#include <max_min_lp_msgs/layered_node_array.h>
#include <max_min_lp_msgs/server_to_robots.h>
#include <max_min_lp_msgs/server_to_robots_array.h>
#include <max_min_lp_core/MaxMinLPCore.hpp>
#include <max_min_lp_simulation/MessageRequest.h>

using namespace std;

class MaxMinLPGreedyRobotNode {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  ros::Subscriber request_sub;

  // Subscriber
  ros::Subscriber m_odom_sub;

  // Clients
  ros::ServiceClient m_client;

  // General node values
  vector<max_min_lp_msgs::general_node> m_gen_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_t_node;
  vector<max_min_lp_msgs::general_node> m_gen_t_node;

  max_min_lp_msgs::server_to_robots_array m_local_info;

  // Variables from the launch file
  int m_num_robot;
  int m_num_target;
  int m_robot_id;
  int m_num_layer; // Number of layers in the layered model
  int m_num_motion_primitive;
  int m_time_interval;
  bool m_verbal_flag;
  double m_epsilon;
  string m_robot_name;

  // Robot info
  geometry_msgs::Pose m_pos;

  // int m_motion_case_rotation[];

public:
  MaxMinLPGreedyRobotNode(); // Constructor

  void updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg); // Update odometry information by subscribing to /robot/odom
  bool initialize();
  vector<geometry_msgs::Pose> computeMotionPrimitives();
  void applyMotionPrimitives(const std_msgs::String::ConstPtr& msg);
};

#endif