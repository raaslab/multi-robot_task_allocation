/**
 * Each robot node for the greedy algorithm.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \07/17/2018
 * Copyright 2018. All Rights Reserved.
 */

#include "max_min_lp_simulation/RobotGreedyJournal.hpp"

#define PHI 3.141592

RobotGreedyJournal::RobotGreedyJournal() :
m_num_robot(1), m_num_target(1), m_robot_id(0), m_robot_name(string("robot_0")),  
m_verbal_flag(false), m_num_motion_primitive(10), m_time_interval(10), m_robot_time(1), 
m_sensing_range(5), m_comm_range(5), m_private_nh("~")
{
	m_private_nh.getParam("num_robot", m_num_robot);
	m_private_nh.getParam("num_target", m_num_target);
	m_private_nh.getParam("robot_id", m_robot_id);
	m_private_nh.getParam("robot_name", m_robot_name);
	m_private_nh.getParam("verbal_flag", m_verbal_flag);
	m_private_nh.getParam("num_motion_primitive", m_num_motion_primitive);
	m_private_nh.getParam("time_interval", m_time_interval);
	m_private_nh.getParam("robot_time", m_robot_time);
	m_private_nh.getParam("sensing_range", m_sensing_range);
	m_private_nh.getParam("comm_range", m_comm_range);

	// Subscribers
	m_odom_sub = m_nh.subscribe("/gazebo/model_states", 1000, &RobotGreedyJournal::updateOdom, this);
	m_request_sub = m_nh.subscribe("/robot_status", 1000, &RobotGreedyJournal::activateRobots, this);
	// In subscribers, compute a communication graph by using the communication range.
	// w_j

	// Publishers
	for (int i = 0; i < m_num_target; i++) {
		m_target_predicted_pose_pub.push_back(m_nh.advertise<geometry_msgs::PoseWithCovariance>("/target_"+boost::lexical_cast<string>(i)+"/predicted_pose", 1));
		m_target_measurement_pose_pub.push_back(m_nh.advertise<geometry_msgs::Pose>("/target_"+boost::lexical_cast<string>(i)+"/measurement_pose", 1));
	}

	m_cur_time =ros::Time::now().toSec();
	m_prev_time =ros::Time::now().toSec();

	m_firt_time_step = true;
	m_moving_time = m_time_interval - m_robot_time*m_num_robot;

	getMotionCaseRotation();
	ROS_INFO("Ready to activate robots.");
}

void RobotGreedyJournal::activateRobots(const std_msgs::String::ConstPtr& msg) {
	while (1) {
		// At every time interval, do the following.
		m_cur_time = ros::Time::now().toSec();

		// Initialization.
		m_measure_target_pos.clear();
		m_predicted_target_pos.clear();
		m_motion_primitive_pose.clear();
		m_c.clear();
		m_w.clear(); // # = number of targets.
		m_w_prime.clear(); // # = number of motion primitives.
		for (int i = 0; i < m_num_target; i++) {
			m_w.push_back(0);
		}

		if (m_cur_time-m_prev_time >= m_time_interval || m_firt_time_step) {
			m_prev_time = ros::Time::now().toSec();

			// Compute c (i.e., the tracking quality) by using the sensing range.
			m_target_odom_client = m_nh.serviceClient<max_min_lp_simulation::GetOdom>("/target_odom_request");
			max_min_lp_simulation::GetOdom srv;

			if (m_target_odom_client.call(srv)) {
				m_measure_target_pos = srv.response.return_target_odom;

				ROS_INFO("\nTarget prediction starts.");

				// Predict next locations of targets.
				int n = 2; // Number of states
				int m = 2; // Number of measurements

				if (m_firt_time_step) {
					// Initialization of kalman filters.
					for (int i = 0; i < m_num_target; i++) {
						Eigen::MatrixXd A(n, n); // System dynamics matrix
						Eigen::MatrixXd C(m, n); // Output matrix
						Eigen::MatrixXd B(n, n); // Control-input model
						Eigen::MatrixXd Q(n, n); // Process noise covariance
						Eigen::MatrixXd R(m, m); // Measurement noise covariance
						Eigen::MatrixXd P(n, n); // Estimate error covariance

						A << 1, 0, 0, 1;
						C << 1, 0, 0, 1;
						B << m_time_interval, 0, 0, m_time_interval;

						Q << .1, 0, 0, .1;
						R << .05, 0, 0, .05;
						P << .05, 0, 0, .05;
						KalmanFilter kf(m_time_interval, A, C, B, Q, R, P);
						
						Eigen::VectorXd x0(n);
						x0 << m_measure_target_pos[i].position.x, m_measure_target_pos[i].position.y;
						kf.init(m_cur_time, x0);

						m_kalman.push_back(kf);

						m_prev_target_pos.push_back(m_measure_target_pos[i]);
						m_predicted_target_pos.push_back(m_measure_target_pos[i]);

						// For debug.
						Eigen::MatrixXd P_extended(n+1, n+1);
						for (int j = 0; j < P_extended.rows(); j++) {
							for (int k = 0; k < P_extended.cols(); k++) {
								if (j < P_extended.rows()-1 && k < P_extended.cols()-1) {
									P_extended(j,k) = P(j,k);
								}
								else {
									P_extended(j,k) = 0;
								}
							}
						}

						geometry_msgs::PoseWithCovariance pred_target_pose;
						pred_target_pose.pose = m_measure_target_pos[i];
						int cov_value_index = 0;
						for (int j = 0; j < P_extended.rows(); j++) {
							for (int k = 0; k < P_extended.cols(); k++) {
								pred_target_pose.covariance[cov_value_index] = P_extended(j,k);
								cov_value_index += 1;
							}
						}
						ROS_INFO("Target measurement pose: %.2f, %.2f", m_measure_target_pos[i].position.x, m_measure_target_pos[i].position.y);
						ROS_INFO("Target predcted pose: %.2f, %.2f", pred_target_pose.pose.position.x, pred_target_pose.pose.position.y);
						m_target_predicted_pose_pub[i].publish(pred_target_pose);
						m_target_measurement_pose_pub[i].publish(m_measure_target_pos[i]);
					}
				}
				else {
					// Update kalman filters.
					for (int i = 0; i < m_num_target; i++) {
						// Compute x and y velocities.
						Eigen::VectorXd U(n); // Control input
						double v_x = (m_measure_target_pos[i].position.x-m_prev_target_pos[i].position.x)/m_time_interval;
						double v_y = (m_measure_target_pos[i].position.y-m_prev_target_pos[i].position.y)/m_time_interval;
						U << v_x, v_y;
						ROS_INFO("Control input = %.2f, %.2f", v_x, v_y);

						Eigen::VectorXd x_prev(n);
						x_prev << m_prev_target_pos[i].position.x, m_prev_target_pos[i].position.y;
						m_kalman[i].update(x_prev, U);

						Eigen::VectorXd x_predicted(n);
						Eigen::MatrixXd P(n, n);
						x_predicted = m_kalman[i].state();
						P = m_kalman[i].covariance();

						geometry_msgs::Pose predicted_target_pose;
						predicted_target_pose.position.x = x_predicted[0];
						predicted_target_pose.position.y = x_predicted[1];
						predicted_target_pose.position.z = 0;

						m_predicted_target_pos.push_back(predicted_target_pose);

						// For debug.
						Eigen::MatrixXd P_extended(n+1, n+1);
						for (int j = 0; j < P_extended.rows(); j++) {
							for (int k = 0; k < P_extended.cols(); k++) {
								if (j < P_extended.rows()-1 && k < P_extended.cols()-1) {
									P_extended(j,k) = P(j,k);
								}
								else {
									P_extended(j,k) = 0;
								}
							}
						}

						geometry_msgs::PoseWithCovariance pred_target_pose;
						pred_target_pose.pose = predicted_target_pose;
						int cov_value_index = 0;
						for (int j = 0; j < P_extended.rows(); j++) {
							for (int k = 0; k < P_extended.cols(); k++) {
								pred_target_pose.covariance[cov_value_index] = P_extended(j,k);
								cov_value_index += 1;
							}
						}
						ROS_INFO("Target measurement pose: %.2f, %.2f", m_measure_target_pos[i].position.x, m_measure_target_pos[i].position.y);
						ROS_INFO("Target predcted pose: %.2f, %.2f", pred_target_pose.pose.position.x, pred_target_pose.pose.position.y);
						m_target_predicted_pose_pub[i].publish(pred_target_pose);
						m_target_measurement_pose_pub[i].publish(m_measure_target_pos[i]);
					}

					m_prev_target_pos.clear();
					m_prev_target_pos = m_measure_target_pos;
				}

				m_firt_time_step = false;
			}
			else {
				ROS_ERROR("ERROR: Targets are not properly obtained.");
			}

			ROS_INFO("Target prediction finished.");

			while (1) { // Wait for predecessor robots to compute.
				m_cur_time = ros::Time::now().toSec();
				if (m_cur_time-m_prev_time >= m_robot_time*m_robot_id) {
					break;
				}
			}

			ROS_INFO("Local computaion of Robot %d starts.", m_robot_id);

			// Obtain motion primitives.
			m_motion_primitive_pose = computeMotionPrimitives();

			for (int i = 0; i < m_num_motion_primitive; i++) {
				// Compute c values and consider a sensing graph.
				vector<double> c_for_primitive;
				for (int j = 0; j < m_num_target; j++) {
					double dist_target_robot = sqrt((m_pos.position.x-m_predicted_target_pos[j].position.x)*(m_pos.position.x-m_predicted_target_pos[j].position.x)
						+(m_pos.position.y-m_predicted_target_pos[j].position.y)*(m_pos.position.y-m_predicted_target_pos[j].position.y));
					// if (dist_target_robot >= m_sensing_range) { // Outside the FoV.
					// 	c_for_primitive.push_back(0);
					// }
					// else { // Inside the FoV.
						double dist_target_primitive = sqrt((m_motion_primitive_pose[i].position.x-m_predicted_target_pos[j].position.x)*
							(m_motion_primitive_pose[i].position.x-m_predicted_target_pos[j].position.x)
							+(m_motion_primitive_pose[i].position.y-m_predicted_target_pos[j].position.y)*
							(m_motion_primitive_pose[i].position.y-m_predicted_target_pos[j].position.y));
						c_for_primitive.push_back(1/dist_target_primitive);
					// }
				}

				m_c.push_back(c_for_primitive);
			}

			// Publishers
			// w_j

			ROS_INFO("Local computaion of Robot %d finished.", m_robot_id);
			
			while (1) { // Wait for sucessor robots to compute.
				m_cur_time = ros::Time::now().toSec();
				if (m_cur_time-m_prev_time >= m_robot_time*(m_num_robot-m_robot_id)) {
					break;
				}
			}

			// Moving over a selected motion primitive starts.
			ROS_INFO("Robot %d starts moving.", m_robot_id);
		}
	}
}

void RobotGreedyJournal::updateOdom(const gazebo_msgs::ModelStates::ConstPtr& msg) {
	int size_msg = m_num_robot + m_num_target + 1; // 1 is for 'ground plane' in gazebo.
	int id = -1;
	for (int i = 0; i < size_msg; i++) {
		if (strcmp(msg->name[i].c_str(), m_robot_name.c_str()) == 0) {
			id = i;
		}
	}
	if (id != -1) {
		m_pos = msg->pose[id];
	}
}

vector<geometry_msgs::Pose> RobotGreedyJournal::computeMotionPrimitives() {
	if (m_verbal_flag) {
		ROS_INFO("ROBOT %d is in the computeMotionPrimitives() step", m_robot_id);
	}
	tf::Quaternion q(m_pos.orientation.x, m_pos.orientation.y, m_pos.orientation.z, m_pos.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	yaw = yaw * 180 / PHI;

	vector<geometry_msgs::Pose> motion_primitive;
	double x_new, y_new;
	double orientation;
	
	for (int i = 0; i < m_num_motion_primitive; i++) {
		geometry_msgs::Pose motion_primitive_instance;
		orientation = yaw + 3.42 * m_motion_case_rotation[i]; // 3.42 is degree.
		if (orientation > 180) {
			orientation -= 360;
		}
		else if (orientation < -180) {
			orientation += 360;
		}
		// This is hard-coded.
		if (orientation >= 0 && orientation < 90) {
			x_new = m_pos.position.x + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* cos(orientation * PHI / 180);
			y_new = m_pos.position.y + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* sin(orientation * PHI / 180);
		}
		else if (orientation >= 90 && orientation < 180) {
			x_new = m_pos.position.x - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* cos((180 - orientation) * PHI / 180);
			y_new = m_pos.position.y + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* sin((180 - orientation) * PHI / 180);
		}
		else if (orientation < 0 && orientation >= -90) {
			x_new = m_pos.position.x + 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* cos((-1) * orientation * PHI / 180);
			y_new = m_pos.position.y - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* sin((-1) * orientation * PHI / 180);
		}
		else if (orientation < -90 && orientation >= -180) {
			x_new = m_pos.position.x - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* cos((180 + orientation) * PHI / 180);
			y_new = m_pos.position.y - 0.06 * (m_time_interval - abs(m_motion_case_rotation[i])) 
					* sin((180 + orientation) * PHI / 180);
		}

		motion_primitive_instance.position.x = x_new;
		motion_primitive_instance.position.y = y_new;
		motion_primitive_instance.orientation.w = orientation;

		if (m_verbal_flag) {
			ROS_INFO("ROBOT %d : %d'th motion primitive = (%f, %f)", m_robot_id, i+1, x_new, y_new);
		}

		motion_primitive.push_back(motion_primitive_instance);
	}

	return motion_primitive;
}

void RobotGreedyJournal::getMotionCaseRotation() {
	ROS_INFO("In getMotionCaseRotation()");
	int num_rotation;
	if (m_num_motion_primitive%2 == 0) {
		num_rotation = m_num_motion_primitive/2;
	}
	else {
		num_rotation = (m_num_motion_primitive-1)/2;
	}

	double rotation_time = m_moving_time/2;
	int resol_rotation = floor(rotation_time/num_rotation);

	for (int i = 0; i < m_num_motion_primitive; i++) {
		if (m_num_motion_primitive%2 == 0) {
			if (i >= num_rotation) {
				m_motion_case_rotation.push_back((num_rotation-i-1)*resol_rotation);
			}
			else {
				m_motion_case_rotation.push_back((num_rotation-i)*resol_rotation);
			}
		}
		else {
			m_motion_case_rotation.push_back((num_rotation-i)*resol_rotation);
		}

		// Check if the rotation is cw, ccw or stationary.
		if (m_motion_case_rotation[i] > 0) {
			m_check_rotation_direction.push_back(1);
		}
		else if (m_motion_case_rotation[i] < 0) {
			m_check_rotation_direction.push_back(-1);
		}
		else {
			m_check_rotation_direction.push_back(0);
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_greedy_journal");

	RobotGreedyJournal rg;

	ros::spin();
	return 0;
}