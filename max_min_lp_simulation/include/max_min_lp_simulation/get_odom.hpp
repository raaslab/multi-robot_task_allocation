#ifndef GETODOM_HPP_
#define GETODOM_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <boost/lexical_cast.hpp>
#include <max_min_lp_simulation/GetOdom.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

using namespace std;

class get_odom {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Subscribers
  ros::Subscriber m_odom_sub;

  // Publishers
  ros::Publisher m_general_node_pub;

  // Services
  ros::ServiceServer m_odom_service;

  string m_robot_name;
  int m_num_robot;
  int m_num_target;
  int m_robot_id;

  geometry_msgs::Pose m_pos;

public:
  get_odom(); // Constructor

  void updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg); // Update odometry information by subscribing to /robot/odom
  bool return_odom(max_min_lp_simulation::GetOdom::Request &req, max_min_lp_simulation::GetOdom::Response &res);
};

#endif