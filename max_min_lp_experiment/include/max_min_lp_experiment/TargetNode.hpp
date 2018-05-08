#ifndef TARGETNODE_HPP_
#define TARGETNODE_HPP_

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

using namespace std;

class TargetNode {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Services
  ros::ServiceServer m_request_service;

  // Params from the launch file
  int m_time_period;

  // Target information
  int m_num_target;
  vector<int> m_target_id;
  vector<float> m_target_pose_x;
  vector<float> m_target_pose_y;
  vector<float> m_target_velocity;
  vector<float> m_target_orientation;

public:
  TargetNode(); // Constructor

  bool applyTargetMotion(max_min_lp_experiment::TargetRequest::Request &req, max_min_lp_experiment::TargetRequest::Response &res);
  void targetInitialize();
};

#endif