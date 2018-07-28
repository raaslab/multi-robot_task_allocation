#ifndef RobotGreedyJournal_HPP_
#define RobotGreedyJournal_HPP_

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
#include <max_min_lp_simulation/get_odom.hpp>
#include <max_min_lp_simulation/apply_motion_primitive.hpp>
#include <max_min_lp_simulation/MotionPrimitiveRequest.h>
#include <max_min_lp_simulation/GetOdom.h>
#include <max_min_lp_simulation/MoveRobot.h>

using namespace std;

class RobotGreedyJournal {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Subscribers
  ros::Subscriber m_odom_sub;

  // Clients
  ros::ServiceClient m_target_odom_client;

  // Variables from the launch file
  int m_num_robot;
  int m_num_target;
  int m_robot_id;
  int m_num_motion_primitive;
  int m_time_interval;
  int m_robot_time;
  double m_sensing_range;
  double m_comm_range;
  bool m_verbal_flag;
  string m_robot_name;

  // Variables for each robot
  vector<double> m_c;
  vector<double> m_w;

  // Robot info
  geometry_msgs::Pose m_pos;

  // Target info of each time-step
  vector<geometry_msgs::Pose> m_cur_target_pos;

public:
  RobotGreedyJournal(); // Constructor

  void updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg); // Update odometry information by subscribing to /robot/odom
  bool initialize();
};

#endif