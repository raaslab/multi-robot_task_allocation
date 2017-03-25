#ifndef MAXMINLPGREEDYCENTRALNODE_HPP_
#define MAXMINLPGREEDYCENTRALNODE_HPP_

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
#include <max_min_lp_msgs/server_to_robots.h>
#include <max_min_lp_msgs/server_to_robots_array.h>
#include <max_min_lp_msgs/target_node.h>
#include <max_min_lp_msgs/primitive_node.h>
#include <max_min_lp_simulation/get_odom.hpp>
#include <max_min_lp_simulation/GetOdom.h>
#include <max_min_lp_simulation/MessageRequest.h>
#include <max_min_lp_simulation/MotionPrimitiveRequest.h>
#include <max_min_lp_simulation/GetTotalNumTarget.h>

using namespace std;

class MaxMinLPGreedyCentralNode {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Publishers
  // ros::Publisher m_request_pub;
  // ros::Publisher m_general_node_pub;
  // ros::Publisher m_layered_node_pub;
  ros::Publisher m_response_to_robot_pub;

  // Subscribers
  ros::Subscriber m_target_1_sub;
  ros::Subscriber m_target_2_sub;
  ros::Subscriber m_target_3_sub;

  ros::Subscriber m_comm_graph_by_robots_sub;

  // Services
  ros::ServiceServer m_service;
  ros::ServiceServer m_primitive_service;
  ros::ServiceServer m_total_num_target_service;

  // Clients
  ros::ServiceClient m_target_odom_client;

  // Variables from the launch file
  max_min_lp_msgs::server_to_robots_array m_robot_info;
  int m_num_robot;
  int m_num_target;
  int m_fov;
  int m_num_motion_primitive;

  int m_send_robot_id;
  int m_request_robot_id;
  bool m_check_request_send;

  // Motion primitives info
  vector<int> m_primitive_id;
  vector<float> m_primitive_x_pos;
  vector<float> m_primitive_y_pos;

  // Target info of each time-step
  vector<geometry_msgs::Pose> m_temp_target_pos;
  vector<string> m_temp_target_name;

  // Target info when consensus of robots is fulfilled
  vector<int> m_target_id;
  vector<float> m_target_x_pos;
  vector<float> m_target_y_pos;

  vector<vector<int> > m_primitives_to_targets; // Motion primitives to targets
  vector<vector<int> > m_targets_to_primitives; // Targets to motion primitives

  vector<int> m_target_index_used;

  vector<int> m_optimal_primitive_id;

  // For text outputs
  ofstream m_outputFile;

  int m_check_finish_action_apply;
  int m_time_step;

  int m_total_num_target_covered;

public:
  MaxMinLPGreedyCentralNode(); // Constructor
  ~MaxMinLPGreedyCentralNode(){
    m_outputFile.close();
  }

  bool initialize(max_min_lp_simulation::MessageRequest::Request &req, max_min_lp_simulation::MessageRequest::Response &res);
  bool sendMotionPrimitive(max_min_lp_simulation::MotionPrimitiveRequest::Request &req, max_min_lp_simulation::MotionPrimitiveRequest::Response &res);
  void applySequentialLocalAlgorithm(const std_msgs::String::ConstPtr& msg);
  bool get_total_num_target(max_min_lp_simulation::GetTotalNumTarget::Request &req, max_min_lp_simulation::GetTotalNumTarget::Response &res);

  void updatePose(const gazebo_msgs::ModelStates::ConstPtr& msg, int target_id); // Update pose information
};

#endif