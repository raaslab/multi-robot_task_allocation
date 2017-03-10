/**
 * Central node for the simulation
 * Central node only links between a set of motion primitives and a set of targets.
 * Important note: in this code a robot is used to take into consideration of the reduction of the step 1 of local algorithm. 
 * Thus, a robot here does not indicate a real robot but an element of a real robot. Instead, ROBOT is used for a real robot.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \02/27/2017
 * Copyright 2017. All Rights Reserved.
 */

#include "max_min_lp_simulation/apply_motion_primitive.hpp"

apply_motion_primitive::apply_motion_primitive() : 
m_num_robot(1), m_num_target(1), m_robot_id(1), m_robot_name(string("robot_1")), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("robot_id", m_robot_id);
	m_private_nh.getParam("robot_name", m_robot_name);
}

bool apply_motion_primitive::initialize() {
}