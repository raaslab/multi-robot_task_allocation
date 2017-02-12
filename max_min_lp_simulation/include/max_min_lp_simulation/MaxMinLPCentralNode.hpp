#ifndef MAXMINLPCENTRALNODE_HPP_
#define MAXMINLPCENTRALNODE_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <max_min_lp_msgs/server_to_robots.h>
#include <max_min_lp_msgs/server_to_robots_array.h>
#include <max_min_lp_simulation/MessageRequest.h>

using namespace std;

class MaxMinLPCentralNode {  
private:
  ros::NodeHandle m_nh;

  // Publishers
  ros::Publisher m_request_pub;
  ros::Publisher m_general_node_pub;
  ros::Publisher m_layered_node_pub;

  // Subscribers
  ros::Subscriber m_test_sub;

  // Services
  ros::ServiceServer m_service;

  // Variables from the launch file
  max_min_lp_msgs::server_to_robots_array m_robot_info;
  int m_num_robot;

  int m_send_robot_id;
  int m_request_robot_id;

public:
  MaxMinLPCentralNode(); // Constructor

  bool initialize(max_min_lp_simulation::MessageRequest::Request &req, max_min_lp_simulation::MessageRequest::Response &res);

  void updateOdom(const nav_msgs::Odometry::ConstPtr& msg); // Update odometry information by subscribing to /robot/odom
};

#endif