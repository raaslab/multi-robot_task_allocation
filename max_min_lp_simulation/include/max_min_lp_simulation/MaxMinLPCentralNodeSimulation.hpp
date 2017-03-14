#ifndef MAXMINLPCENTRALNODESIMULATION_HPP_
#define MAXMINLPCENTRALNODESIMULATION_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <max_min_lp_core/MaxMinLPSequentialCore.hpp>
#include <max_min_lp_msgs/general_node.h>
#include <max_min_lp_msgs/general_node_array.h>
#include <max_min_lp_msgs/server_to_robots.h>
#include <max_min_lp_msgs/server_to_robots_array.h>
#include <max_min_lp_msgs/target_node.h>
#include <max_min_lp_msgs/primitive_node.h>
#include <max_min_lp_simulation/get_odom.hpp>
#include <max_min_lp_simulation/GetOdom.h>
#include <max_min_lp_simulation/MessageRequest.h>
#include <max_min_lp_simulation/MotionPrimitiveRequest.h>

using namespace std;

class MaxMinLPCentralNodeSimulation {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Subscribers
  ros::Subscriber m_target_1_sub;
  ros::Subscriber m_target_2_sub;
  ros::Subscriber m_target_3_sub;
  ros::Subscriber m_target_4_sub;
  ros::Subscriber m_target_5_sub;
  ros::Subscriber m_target_6_sub;
  ros::Subscriber m_target_7_sub;
  ros::Subscriber m_target_8_sub;
  ros::Subscriber m_target_9_sub;
  ros::Subscriber m_target_10_sub;
  ros::Subscriber m_target_11_sub;
  ros::Subscriber m_target_12_sub;
  ros::Subscriber m_target_13_sub;
  ros::Subscriber m_target_14_sub;
  ros::Subscriber m_target_15_sub;
  ros::Subscriber m_target_16_sub;
  ros::Subscriber m_target_17_sub;
  ros::Subscriber m_target_18_sub;
  ros::Subscriber m_target_19_sub;
  ros::Subscriber m_target_20_sub;
  ros::Subscriber m_target_21_sub;
  ros::Subscriber m_target_22_sub;
  ros::Subscriber m_target_23_sub;
  ros::Subscriber m_target_24_sub;
  ros::Subscriber m_target_25_sub;
  ros::Subscriber m_target_26_sub;
  ros::Subscriber m_target_27_sub;
  ros::Subscriber m_target_28_sub;
  ros::Subscriber m_target_29_sub;
  ros::Subscriber m_target_30_sub;

  ros::Subscriber m_comm_graph_by_robots_sub;

  // Publishers
  ros::Publisher m_general_node_pub;
  ros::Publisher m_layered_node_pub;
  ros::Publisher m_response_to_robot_pub;

  // Services
  ros::ServiceServer m_service;
  ros::ServiceServer m_primitive_service;

  // Clients
  ros::ServiceClient m_target_odom_client;

  // Variables from the launch file
  max_min_lp_msgs::server_to_robots_array m_robot_info;
  int m_num_robot;
  int m_num_target;
  int m_num_motion_primitive;
  int m_num_layer;
  float m_fov;
  double m_epsilon;
  string m_objective_option;
  bool m_verbal_flag;
  bool m_verbal_local_flag;

  int m_send_robot_id;
  int m_request_robot_id;
  bool m_check_request_send;
  bool m_check_apply_sequential_send;
  int m_check_finish_action_apply;

  bool m_ready_to_send;

  int m_time_step;

  // For reduction
  vector<float> m_constraint_value; // This value is used for reduction i.e., 2/|V_i|.

  //// ROBOT ////
  // ROBOT robot
  vector<int> m_ROBOT_num_robot;
  vector<int> m_prev_accumulate_robot;
  int m_num_survived_robot;
  vector<float> m_ROBOT_x_pos;
  vector<float> m_ROBOT_y_pos;

  // ROBOT motion primitives
  vector<int> m_ROBOT_num_motion_primitive;
  vector<int> m_prev_accumulate_motion_primitive;
  int m_num_survived_motion_primitive;

  // ROBOT neighbor hop info
  vector<vector<vector<int> > > m_ROBOT_neighbor;
  vector<vector<int> > m_ROBOT_assign_targets;
  vector<int> m_max_neighbor_hop;

  //// robot ////
  // robot info
  vector<int> m_robot_id;

  //// motion primitives ////
  vector<int> m_primitive_id;
  vector<int> m_primitive_original_id;
  vector<float> m_primitive_x_pos;
  vector<float> m_primitive_y_pos;
  vector<float> m_dist_primitive_to_target;

  //// target ////
  // Target info of each time-step obtained from gazebo
  vector<geometry_msgs::Pose> m_temp_target_pos;
  vector<string> m_temp_target_name;

  // Target info when consensus of robots is fulfilled
  vector<int> m_target_id;
  vector<float> m_target_x_pos;
  vector<float> m_target_y_pos;
  vector<bool> m_target_observed;

  vector<vector<int> > m_ROBOTs_to_targets; // ROBOTs to targets
  vector<vector<int> > m_robots_to_primitives; // Robots to motion primitives
  vector<vector<int> > m_primitives_to_robots; // Motion primitives to robots
  vector<vector<int> > m_primitives_to_targets; // Motion primitives to targets
  vector<vector<int> > m_targets_to_primitives; // Targets to motion primitives

  vector<vector<float> > m_primitives_to_targets_weight; // Weights of motion primitives to targets
  vector<vector<float> > m_targets_to_primitives_weight; // Weights of targets to motion primitives

  // Send optimal motion primitives to each robot
  vector<int> m_optimal_primitive_id;
  vector<int> m_optimal_primitive_id_for_plot;

  // General node values
  vector<max_min_lp_msgs::general_node> m_gen_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_t_node;
  vector<max_min_lp_msgs::general_node> m_gen_t_node;

  // For text outputs
  ofstream m_target_outputFile;
  ofstream m_outputFile;

public:
  MaxMinLPCentralNodeSimulation(); // Constructor
  ~MaxMinLPCentralNodeSimulation() {
    m_target_outputFile.close();
    m_outputFile.close();
  }

  bool initialize(max_min_lp_simulation::MessageRequest::Request &req, max_min_lp_simulation::MessageRequest::Response &res);
  bool sendMotionPrimitive(max_min_lp_simulation::MotionPrimitiveRequest::Request &req, max_min_lp_simulation::MotionPrimitiveRequest::Response &res);
  void updatePose(const gazebo_msgs::ModelStates::ConstPtr& msg, int target_id); // Update pose information
  vector<max_min_lp_msgs::general_node> buildGeneralNode(string option);
  void applySequentialLocalAlgorithm(const std_msgs::String::ConstPtr& msg);
};

#endif