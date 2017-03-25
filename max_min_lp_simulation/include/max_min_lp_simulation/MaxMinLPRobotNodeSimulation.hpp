#ifndef MAXMINLPROBOTNODESIMULATION_HPP_
#define MAXMINLPROBOTNODESIMULATION_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <tf/transform_datatypes.h>
#include <max_min_lp_simulation/get_odom.hpp>
#include <max_min_lp_simulation/apply_motion_primitive.hpp>
#include <max_min_lp_msgs/general_node.h>
#include <max_min_lp_msgs/general_node_array.h>
#include <max_min_lp_msgs/layered_node.h>
#include <max_min_lp_msgs/layered_node_array.h>
#include <max_min_lp_msgs/server_to_robots.h>
#include <max_min_lp_msgs/server_to_robots_array.h>
#include <max_min_lp_core/MaxMinLPDecentralizedCore.hpp>
#include <max_min_lp_simulation/MessageRequest.h>
#include <max_min_lp_simulation/MotionPrimitiveRequest.h>
#include <max_min_lp_simulation/GetOdom.h>
#include <max_min_lp_simulation/MoveRobot.h>

using namespace std;

class MaxMinLPRobotNodeSimulation {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  ros::Subscriber request_sub;

  // Publishers
  ros::Publisher m_general_node_pub;
  ros::Publisher m_layered_node_pub;
  ros::Publisher m_response_to_server_pub;
  ros::Publisher m_cmd_vel_robot_pub;

  // Subscriber
  ros::Subscriber m_odom_sub;

  // Clients
  ros::ServiceClient m_client;
  ros::ServiceClient m_primitive_client;
  ros::ServiceClient m_odom_client;
  ros::ServiceClient m_move_client;

  // Variables from the launch file
  int m_num_robot;
  int m_num_target;
  int m_robot_id;
  int m_num_layer; // Number of layers in the layered model
  int m_num_motion_primitive;
  int m_time_interval;
  bool m_verbal_flag;
  double m_epsilon;
  string m_robot_name;

  // Optimal motion primitive that is applied at the end of algorithm at each time
  int m_count_time_interval;
  int m_selected_primitive_id;
  vector<int> m_motion_case_rotation;
  vector<geometry_msgs::Pose> m_motion_primitive_pose;
  vector<int> m_check_rotation_direction;

  // Robot info
  geometry_msgs::Pose m_pos;

  // For reduction
  vector<float> m_constraint_value;

  // Local info
  int m_max_neighbor_hop;
  vector<int> m_num_neighbors_at_each_hop;
  vector<int> m_num_new_targets_at_each_hop;

  // ROBOT robot
  vector<int> m_ROBOT_num_robot;
  vector<int> m_prev_accumulate_robot;
  int m_num_survived_robot;

  // ROBOT motion primitives
  vector<int> m_ROBOT_num_motion_primitive;
  vector<int> m_prev_accumulate_motion_primitive;
  int m_num_survived_motion_primitive;

  vector<max_min_lp_msgs::general_node> m_gen_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_t_node;
  vector<max_min_lp_msgs::general_node> m_gen_t_node;

  // For debugging purpose
  int m_count_initialize_func;

  ofstream m_robot_outputFile;

  bool m_check_start;

  int count_computeMotionPrimitives;
  int * m_random_number_1;
  int * m_random_number_2;
  int * m_random_number_3;
  int * m_random_number_4;
  int * m_random_number_5;

public:
  MaxMinLPRobotNodeSimulation(); // Constructor
  ~MaxMinLPRobotNodeSimulation() {
    m_robot_outputFile.close();
    delete[] m_random_number_1;
    delete[] m_random_number_2;
    delete[] m_random_number_3;
    delete[] m_random_number_4;
    delete[] m_random_number_5;
  }

  void updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg); // Update odometry information by subscribing to /robot/odom
  bool initialize();
  bool getMotionPrimitive();
  void applyMotionPrimitives(const std_msgs::String::ConstPtr& msg);
  vector<geometry_msgs::Pose> computeMotionPrimitives();
};

#endif