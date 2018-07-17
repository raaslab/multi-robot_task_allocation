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

  // Subscriber
  ros::Subscriber m_request_sub;
  ros::Subscriber m_odom_sub;

  // Clients
  ros::ServiceClient m_client;
  ros::ServiceClient m_primitive_client;
  ros::ServiceClient m_move_client;

  // General node values
  vector<max_min_lp_msgs::general_node> m_gen_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_t_node;
  vector<max_min_lp_msgs::general_node> m_gen_t_node;

  max_min_lp_msgs::server_to_robots_array m_local_info;

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

  // Robot info
  geometry_msgs::Pose m_pos;

  int m_count_initialize_func;

  // int m_motion_case_rotation[];

  // Optimal motion primitive that is applied at the end of algorithm at each time
  int m_selected_primitive_id;
  vector<int> m_motion_case_rotation;
  vector<int> m_check_rotation_direction;
  vector<geometry_msgs::Pose> m_motion_primitive_pose;

  ofstream m_robot_outputFile;

  int count_computeMotionPrimitives;
  int * m_random_number_1;
  int * m_random_number_2;
  int * m_random_number_3;
  int * m_random_number_4;
  int * m_random_number_5;

public:
  RobotGreedyJournal(); // Constructor
  ~RobotGreedyJournal() {
    m_robot_outputFile.close();
    delete[] m_random_number_1;
    delete[] m_random_number_2;
    delete[] m_random_number_3;
    delete[] m_random_number_4;
    delete[] m_random_number_5;
  }

  void updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg); // Update odometry information by subscribing to /robot/odom
  bool initialize();
  vector<geometry_msgs::Pose> computeMotionPrimitives();
  void applyMotionPrimitives(const std_msgs::String::ConstPtr& msg);
};

#endif