#ifndef MAXMINLPSEQUENTIALCORE_HPP_
#define MAXMINLPSEQUENTIALCORE_HPP_

#include <iostream>
#include <sstream>
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

using namespace std;

namespace max_min_lp_core {

struct TreeStruct {
  vector< vector<string> > robot_node_id;
  vector< vector<string> > red_node_id;
  vector< vector<string> > blue_node_id;
  vector< vector<string> > target_node_id;
  int tree_depth;
};

class LayeredClass {
private:
  string m_current;
  int m_layer;
  string m_state;
public:
  LayeredClass(string _current, int _layer, string _state) :
  m_current(_current), m_layer(_layer), m_state(_state) {}

  bool operator<(const LayeredClass &right) const {
    if (m_current == right.m_current) {
      if (m_layer == right.m_layer) {
        return m_state < right.m_state;
      }
      else {
        return m_layer < right.m_layer;
      }
    }
    else {
      return m_current < right.m_current;
    }
  }
};

class MaxMinLPSequentialCore {  
private:
  // General node values
  vector<max_min_lp_msgs::general_node> m_gen_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_r_node;
  vector<max_min_lp_msgs::general_node> m_gen_p_t_node;
  vector<max_min_lp_msgs::general_node> m_gen_t_node;

  // Layered node values
  vector<max_min_lp_msgs::layered_node> m_lay_robot_node;
  vector<max_min_lp_msgs::layered_node> m_lay_red_node;
  vector<max_min_lp_msgs::layered_node> m_lay_blue_node;
  vector<max_min_lp_msgs::layered_node> m_lay_target_node;

  map<LayeredClass, max_min_lp_msgs::layered_node> m_layered_map;

  TreeStruct * m_red_tree;

  int m_num_red_layer_zero;

  //// Variables from the launch file
  int m_num_layer; // Number of layers in the layered model
  bool m_verbal_flag;
  double m_epsilon;
  int m_max_neighbor_hop;
  vector<int> m_num_neighbors_at_each_hop;

  // ROBOT robot
  vector<int> m_ROBOT_num_robot;
  vector<int> m_prev_accumulate_robot;
  int m_num_survived_robot;

  // ROBOT motion primitives
  vector<int> m_ROBOT_num_motion_primitive;
  vector<int> m_prev_accumulate_motion_primitive;
  int m_num_survived_motion_primitive;

  vector<float> m_constraint_value;

public:
  // Constructor
  MaxMinLPSequentialCore();
  MaxMinLPSequentialCore(vector<max_min_lp_msgs::general_node>& _gen_r_node, vector<max_min_lp_msgs::general_node>& _gen_p_r_node, 
    vector<max_min_lp_msgs::general_node>& _gen_p_t_node, vector<max_min_lp_msgs::general_node>& _gen_t_node, int _num_layer, 
    bool _verbal_flag, double _epsilon, vector<int> _ROBOT_num_robot, vector<int> _prev_accumulate_robot, int _num_survived_robot, 
    vector<int> _ROBOT_num_motion_primitive, vector<int> _prev_accumulate_motion_primitive, int _num_survived_motion_primitive, 
    vector<float> _constraint_value);
  // Destructor
  ~MaxMinLPSequentialCore() {
    // delete[] m_red_tree;
  }

  void convertSequentialLayeredMaxMinLP(); // Converting the general graph into the layered graph. Step 2
  void applyLocalAlgorithm(); // Step 3

  map<LayeredClass, max_min_lp_msgs::layered_node>::iterator getMapPointer(string _current, int _layer, string _state);
  void getRedTreeStruct(TreeStruct * _red_tree, string _current, int _layer, string _state);
  bool computeRecursive(int _count_red_layer_zero, float _minimum_g_t );

  vector<max_min_lp_msgs::layered_node> getRobotLayeredNode() {
    return m_lay_robot_node;
  }
  vector<max_min_lp_msgs::layered_node> getRedLayeredNode() {
   return m_lay_red_node; 
  }
  vector<max_min_lp_msgs::layered_node> getBlueLayeredNode() {
    return m_lay_blue_node;
  }
  vector<max_min_lp_msgs::layered_node> getTargetLayeredNode() {
    return m_lay_target_node;
  }
};

} // namespace

#endif