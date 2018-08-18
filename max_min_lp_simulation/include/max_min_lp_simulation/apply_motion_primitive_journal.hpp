#ifndef APPLYMOTIONPRIMITIVEJOURNAL_HPP_
#define APPLYMOTIONPRIMITIVEJOURNAL_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <max_min_lp_simulation/get_odom.hpp>
#include <max_min_lp_simulation/GetOdom.h>
#include <max_min_lp_simulation/MoveRobot.h>
#include <boost/lexical_cast.hpp>
#include <tf/transform_datatypes.h>
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
  ros::Publisher m_cmd_vel_robot_pub;

  // Services
  ros::ServiceServer m_move_service;

  // Clients
  ros::ServiceClient m_odom_client;

  string m_robot_name;
  int m_num_robot;
  int m_num_target;
  int m_robot_id;
  int m_check_rotation_direction;
  int m_trans_duration;
  int m_ang_duration;
  double m_trans_speed;
  double m_ang_speed;

  geometry_msgs::Pose m_pos;

public:
  apply_motion_primitive(); // Constructor

  bool move_robot(max_min_lp_simulation::MoveRobot::Request &req, max_min_lp_simulation::MoveRobot::Response &res);
};

#endif