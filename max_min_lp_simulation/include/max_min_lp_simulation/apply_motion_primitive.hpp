#ifndef APPLYMOTIONPRIMITIVE_HPP_
#define APPLYMOTIONPRIMITIVE_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <max_min_lp_simulation/get_odom.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

using namespace std;

class apply_motion_primitive {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Subscribers
  ros::Subscriber m_odom_sub;

  // Publishers
  ros::Publisher m_general_node_pub;

  string m_robot_name;
  int m_num_robot;
  int m_num_target;
  int m_robot_id;

  geometry_msgs::Pose m_pos;

public:
  apply_motion_primitive(); // Constructor

  bool initialize();
};

#endif