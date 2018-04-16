#ifndef GREEDYRobotNode_HPP_
#define GREEDYRobotNode_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <tf/transform_datatypes.h>
#include <max_min_lp_experiment/RobotRequest.h>

using namespace std;

class GreedyRobotNode {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Publisher
  ros::Publisher m_after_motion_pub;
  // Subscriber
  ros::Subscriber m_response_sub;
  // Clients
  ros::ServiceClient m_request_client;

  // Variables from the launch file
  int m_num_target;
  int m_robot_id;
  int m_num_motion_primitive;

  // Robot info
  geometry_msgs::Pose m_pos;

  int m_count_robotInitialize_activate;

  // Optimal motion primitive that is applied at the end of algorithm at each time
  int m_selected_primitive_id;
  vector<int> m_motion_case_rotation;
  vector<int> m_check_rotation_direction;
  vector<geometry_msgs::Pose> m_motion_primitive_pose;

public:
  GreedyRobotNode(); // Constructor

  bool robotInitialize();
  vector<geometry_msgs::Pose> computeMotionPrimitives();
  void applyMotionPrimitives(const std_msgs::String::ConstPtr& msg);
};

#endif