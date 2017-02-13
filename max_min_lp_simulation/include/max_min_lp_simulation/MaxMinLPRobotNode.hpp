#ifndef MAXMINLPROBOTNODE_HPP_
#define MAXMINLPROBOTNODE_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/lexical_cast.hpp>
#include <max_min_lp_msgs/general_node.h>
#include <max_min_lp_msgs/general_node_array.h>
#include <max_min_lp_msgs/layered_node.h>
#include <max_min_lp_msgs/layered_node_array.h>
#include <max_min_lp_msgs/server_to_robots.h>
#include <max_min_lp_msgs/server_to_robots_array.h>
#include <max_min_lp_core/MaxMinLPCore.hpp>
#include <max_min_lp_simulation/MessageRequest.h>

using namespace std;

class MaxMinLPRobotNode {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

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
  int m_robot_id;
  string m_robot_name;
  int m_num_layer; // Number of layers in the layered model
  bool m_verbal_flag;
  double m_epsilon;
  int m_num_motion_primitive;

  // Odom info
  nav_msgs::Odometry m_odom;

public:
  MaxMinLPRobotNode(); // Constructor

  void updateOdom(const nav_msgs::Odometry::ConstPtr& msg); // Update odometry information by subscribing to /robot/odom
  bool initialize();
  void computeMotionPrimitives(const ros::TimerEvent& event);
  void updateGraph();
};

#endif