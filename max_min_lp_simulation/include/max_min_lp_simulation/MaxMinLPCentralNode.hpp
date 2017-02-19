#ifndef MAXMINLPCENTRALNODE_HPP_
#define MAXMINLPCENTRALNODE_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <max_min_lp_msgs/general_node.h>
#include <max_min_lp_msgs/general_node_array.h>
#include <max_min_lp_msgs/server_to_robots.h>
#include <max_min_lp_msgs/server_to_robots_array.h>
#include <max_min_lp_msgs/target_node.h>
#include <max_min_lp_msgs/primitive_node.h>
#include <max_min_lp_simulation/MessageRequest.h>

using namespace std;

class MaxMinLPCentralNode {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Publishers
  // ros::Publisher m_request_pub;
  // ros::Publisher m_general_node_pub;
  // ros::Publisher m_layered_node_pub;

  // Subscribers
  ros::Subscriber m_target_1_sub;
  ros::Subscriber m_target_2_sub;
  ros::Subscriber m_target_3_sub;

  // Services
  ros::ServiceServer m_service;

  // Variables from the launch file
  max_min_lp_msgs::server_to_robots_array m_robot_info;
  int m_num_robot;
  int m_num_target;
  int m_num_motion_primitive;
  int m_num_layer;
  int m_fov;
  string m_objective_option;
  bool m_verbal_flag;

  int m_send_robot_id;
  int m_request_robot_id;
  bool m_check_request_send;

  // For reduction
  int m_num_constraints;
  float m_constraint_value; // This value is used for weights between targets and primitives. (A_tp)

  // ROBOT neighbor hop info
  vector<vector<vector<int> > > m_ROBOT_neighbor;
  vector<vector<int> > m_ROBOT_assign_targets;
  vector<int> m_max_neighbor;

  // Robot info
  vector<int> m_robot_id;

  // Motion primitives info
  vector<int> m_primitive_id;
  vector<float> m_primitive_x_pos;
  vector<float> m_primitive_y_pos;

  // Target info of each time-step obtained from gazebo
  vector<geometry_msgs::Pose> m_temp_target_pos;
  vector<string> m_temp_target_name;

  // Target info when consensus of robots is fulfilled
  vector<int> m_target_id;
  vector<float> m_target_x_pos;
  vector<float> m_target_y_pos;
  vector<bool> m_target_observed;

  vector<vector<int> > m_robots_to_primitives; // Robots to motion primitives
  vector<vector<int> > m_primitives_to_robots; // Motion primitives to robots
  vector<vector<int> > m_primitives_to_targets; // Motion primitives to targets
  vector<vector<int> > m_targets_to_primitives; // Targets to motion primitives

  vector<vector<float> > m_primitives_to_targets_weight; // Weights of motion primitives to targets
  vector<vector<float> > m_targets_to_primitives_weight; // Weights of targets to motion primitives

  // General node values
  vector<max_min_lp_msgs::general_node> m_gen_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_t_node;
  vector<max_min_lp_msgs::general_node> m_gen_t_node;

public:
  MaxMinLPCentralNode(); // Constructor

  bool initialize(max_min_lp_simulation::MessageRequest::Request &req, max_min_lp_simulation::MessageRequest::Response &res);
  void updatePose(const gazebo_msgs::ModelStates::ConstPtr& msg, int target_id); // Update pose information
  vector<max_min_lp_msgs::general_node> buildGeneralNode(string option);
};

#endif