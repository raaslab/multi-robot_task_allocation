#ifndef MAXMINLPDEMO_HPP_
#define MAXMINLPDEMO_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/lexical_cast.hpp>
#include <max_min_lp_msgs/general_node.h>
#include <max_min_lp_msgs/general_node_array.h>
#include <max_min_lp_msgs/layered_node.h>
#include <max_min_lp_msgs/layered_node_array.h>
#include <max_min_lp_core/MaxMinLPCore.hpp>

using namespace std;

class MaxMinLPDemo {  
private:
  ros::NodeHandle m_nh;

  // Publishers
  ros::Publisher m_general_node_pub;
  ros::Publisher m_layered_node_pub;

  // Subscribers
  ros::Subscriber m_test_sub;

  // General node values
  vector<max_min_lp_msgs::general_node> m_gen_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_t_node;
  vector<max_min_lp_msgs::general_node> m_gen_t_node;

  // Variables from the launch file
  string m_input_type;
  string m_output_type;
  int m_num_layer; // Number of layers in the layered model
  int m_num_text_files;
  bool m_verbal_flag;
  double m_epsilon;

  ofstream m_outputFile;

  int m_count;
  bool check_start;

public:
  MaxMinLPDemo(); // Constructor

  void maxMinCallback(const std_msgs::String::ConstPtr& msg);
  void computeLocalAlgorithm(const std_msgs::String::ConstPtr& msg, int num_robot_node, int num_p_t_node, vector<int> p_t_id, vector<int> p_t_loc_deg, vector<vector<int> > p_t_neighbor);
};

#endif