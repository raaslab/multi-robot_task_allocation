#ifndef RobotGreedyJournal_HPP_
#define RobotGreedyJournal_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <tf/transform_datatypes.h>
#include <max_min_lp_simulation/get_odom.hpp>
#include <max_min_lp_simulation/apply_motion_primitive.hpp>
#include <max_min_lp_simulation/MotionPrimitiveRequest.h>
#include <max_min_lp_simulation/GetOdom.h>
#include <max_min_lp_simulation/MoveRobot.h>
#include <max_min_lp_simulation/kalman.hpp>

using namespace std;

class RobotGreedyJournal {  
private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;

  // Subscribers
  ros::Subscriber m_odom_sub;
  ros::Subscriber m_request_sub;

  // Publishers
  vector<ros::Publisher> m_target_predicted_pose_pub;
  vector<ros::Publisher> m_target_measurement_pose_pub;

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
  vector<vector<double> > m_c;
  vector<double> m_w;
  vector<double> m_w_prime;
  vector<KalmanFilter> m_kalman;
  bool m_firt_time_step;
  double m_moving_time;
  double m_cur_time;
  double m_prev_time;

  // Motion primitive-related
  vector<int> m_motion_case_rotation;
  vector<int> m_check_rotation_direction;
  vector<geometry_msgs::Pose> m_motion_primitive_pose;

  // Robot info
  geometry_msgs::Pose m_pos;

  // Target info of each time-step
  vector<geometry_msgs::Pose> m_measure_target_pos;
  vector<geometry_msgs::Pose> m_predicted_target_pos;
  vector<geometry_msgs::Pose> m_prev_target_pos;

public:
  RobotGreedyJournal(); // Constructor

  void activateRobots(const std_msgs::String::ConstPtr& msg);
  void updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg); // Update odometry information by subscribing to /robot/odom
  vector<geometry_msgs::Pose> computeMotionPrimitives();
  void getMotionCaseRotation();
  bool initialize();
};

#endif