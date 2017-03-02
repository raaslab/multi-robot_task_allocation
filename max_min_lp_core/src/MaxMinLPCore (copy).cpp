/**
 * Finds the motion primitive that optimally allocates tracking targets
 * given motion primitives of multiple robots.
 * \Author Yoonchang Sung <yooncs8@vt.edu>
 * \11/22/2016
 * Copyright 2016. All Rights Reserved.
 */

#include "max_min_lp_core/MaxMinLPCore.hpp"

#define MAX_VALUE 10000

namespace max_min_lp_core {

MaxMinLPCore::MaxMinLPCore(vector<max_min_lp_msgs::general_node>& _gen_r_node, vector<max_min_lp_msgs::general_node>& _gen_p_r_node, 
	vector<max_min_lp_msgs::general_node>& _gen_p_t_node, vector<max_min_lp_msgs::general_node>& _gen_t_node, 
	int _num_layer, bool _verbal_flag, double _epsilon) :
m_gen_r_node(_gen_r_node), m_gen_p_r_node(_gen_p_r_node), m_gen_p_t_node(_gen_p_t_node), m_gen_t_node(_gen_t_node),
m_num_layer(_num_layer), m_verbal_flag(_verbal_flag), m_epsilon(_epsilon) {
}

void MaxMinLPCore::convertLayeredMaxMinLP() {
	//Initialization
	m_lay_robot_node.clear();
	m_lay_red_node.clear();
	m_lay_blue_node.clear();
	m_lay_target_node.clear();

	// Robot nodes
	int count_robot_nodes = 0;
	for (vector<max_min_lp_msgs::general_node>::iterator it = m_gen_r_node.begin(); it != m_gen_r_node.end(); ++it) {
		for (int i = 0; i <= m_num_layer; i++) {
			for (int j = 0; j < it->loc_deg; j++) {
				max_min_lp_msgs::layered_node temp_lay_node;
				vector<int> temp_neighbor;
				for (int k = 0; k < it->loc_deg; k++) {
					temp_neighbor.push_back(it->loc_neighbor[k]);
				}
				temp_lay_node.id = it->id;
				temp_lay_node.layer = i;
				temp_lay_node.connected_id = it->loc_neighbor[j];
				temp_lay_node.state = it->type;
				temp_lay_node.f_r = 0;

				if (it->loc_deg == 1) {
					temp_lay_node.loc_deg = 1;
					temp_lay_node.loc_neighbor_id.push_back(it->loc_neighbor[j]);
					temp_lay_node.loc_layer.push_back(i);
					temp_lay_node.loc_connected_id.push_back(it->id);
					temp_lay_node.loc_state.push_back("blue");
					temp_lay_node.edge_weight.push_back((it->loc_edge_weight[j])/2);
				}
				else {
					temp_neighbor.erase (temp_neighbor.begin()+j);
					temp_lay_node.loc_deg = (int)temp_neighbor.size();

					for (vector<int>::iterator itt = temp_neighbor.begin(); itt != temp_neighbor.end(); ++itt) {
						temp_lay_node.loc_neighbor_id.push_back(*itt);
						temp_lay_node.loc_layer.push_back(i);
						temp_lay_node.loc_connected_id.push_back(it->id);
						temp_lay_node.loc_state.push_back("blue");

						for (int l = 0; l < it->loc_deg; l++) {
							if (it->loc_neighbor[l] != *itt) {
								continue;
							}
							temp_lay_node.edge_weight.push_back(it->loc_edge_weight[l]);
						}
					}
				}

				string temp_current = "r"+boost::lexical_cast<string>(temp_lay_node.id);
				string temp_state = "p"+boost::lexical_cast<string>(temp_lay_node.connected_id);

				LayeredClass * temp_class = new LayeredClass(temp_current, i, temp_state);
				m_layered_map[*temp_class] = temp_lay_node;
				delete temp_class;

				m_lay_robot_node.push_back(temp_lay_node);

				if (m_verbal_flag) {
					count_robot_nodes += 1;
					cout<<"---------- Robot node: "<<count_robot_nodes<<" ----------"<<endl;
					cout<<"Current: ("<<temp_current.c_str()<<", "<<i<<", "<<temp_state.c_str()<<"), local degree = "<<temp_lay_node.loc_deg<<endl;
					for (int k = 0; k < temp_neighbor.size(); k++) {
						cout<<"Next: (p"<<temp_lay_node.loc_neighbor_id[k]<<", "<<temp_lay_node.loc_layer[k]<<
							", blue), local weight = "<<temp_lay_node.edge_weight[k]<<endl;
					}
				}
			}
		}
	}

	if (m_verbal_flag) {
		cout<<" "<<endl;
		cout<<"***********************************************************"<<endl;
	}

	// Red nodes (Motion primitive nodes)
	int count_red_nodes = 0;
	m_num_red_layer_zero = 0;
	for (vector<max_min_lp_msgs::general_node>::iterator it = m_gen_p_r_node.begin(); it != m_gen_p_r_node.end(); ++it) {
		for (int i = 0; i <= m_num_layer; i++) {
			max_min_lp_msgs::layered_node temp_lay_node;

			temp_lay_node.id = it->id;
			temp_lay_node.layer = i;
			temp_lay_node.state = it->type;
			temp_lay_node.x_v = 0;
			temp_lay_node.loc_deg = it->loc_deg;

			for (int j = 0; j < it->loc_deg; j++) {
				temp_lay_node.connected_id = it->loc_neighbor[j];
				temp_lay_node.loc_neighbor_id.push_back(it->loc_neighbor[j]);
				temp_lay_node.loc_layer.push_back(i);
				temp_lay_node.loc_connected_id.push_back(it->id);
				temp_lay_node.loc_state.push_back("robot");

				if (m_gen_r_node[it->loc_neighbor[j]-1].loc_deg == 1) {
					temp_lay_node.edge_weight.push_back((it->loc_edge_weight[j])/2);
				}
				else {
					temp_lay_node.edge_weight.push_back(it->loc_edge_weight[j]);
				}
			}

			string temp_current = "p"+boost::lexical_cast<string>(temp_lay_node.id);
			string temp_state = "red";

			LayeredClass * temp_class = new LayeredClass(temp_current, i, temp_state);
			m_layered_map[*temp_class] = temp_lay_node;
			delete temp_class;

			m_lay_red_node.push_back(temp_lay_node);

			if (temp_lay_node.layer == 0) {
				m_num_red_layer_zero += 1;
			}

			if (m_verbal_flag) {
				count_red_nodes += 1;
				cout<<"---------- Red node: "<<count_red_nodes<<" ----------"<<endl;
				cout<<"Current: ("<<temp_current.c_str()<<", "<<i<<", "<<temp_state.c_str()<<"), local degree = "<<temp_lay_node.loc_deg<<endl;
				for (int k = 0; k < it->loc_deg; k++) {
					cout<<"Next: (r"<<temp_lay_node.loc_neighbor_id[k]<<", "<<temp_lay_node.loc_layer[k]<<", p"
						<<boost::lexical_cast<string>(temp_lay_node.id)<<"), local weight = "<<temp_lay_node.edge_weight[k]<<endl;
				}
			}
		}
	}

	if (m_verbal_flag) {
		cout<<" "<<endl;
		cout<<" "<<endl;
		cout<<"***********************************************************"<<endl;
	}

	// Blue nodes (Motion primitive nodes)
	int count_blue_nodes = 0;
	for (vector<max_min_lp_msgs::general_node>::iterator it = m_gen_p_t_node.begin(); it != m_gen_p_t_node.end(); ++it) {
		for (int i = 0; i <= m_num_layer; i++) {
			max_min_lp_msgs::layered_node temp_lay_node;

			temp_lay_node.id = it->id;
			temp_lay_node.layer = i;
			temp_lay_node.state = it->type;
			temp_lay_node.x_v = 0;
			temp_lay_node.loc_deg = it->loc_deg;

			for (int j = 0; j < it->loc_deg; j++) {
				temp_lay_node.connected_id = it->loc_neighbor[j];
				temp_lay_node.loc_neighbor_id.push_back(it->loc_neighbor[j]);
				temp_lay_node.loc_layer.push_back(i+1);
				temp_lay_node.loc_connected_id.push_back(it->id);
				temp_lay_node.loc_state.push_back("target");

				if (m_gen_t_node[it->loc_neighbor[j]-1].loc_deg == 1) {
					temp_lay_node.edge_weight.push_back((it->loc_edge_weight[j])/2);
				}
				else {
					temp_lay_node.edge_weight.push_back(it->loc_edge_weight[j]);
				}
			}

			string temp_current = "p"+boost::lexical_cast<string>(temp_lay_node.id);
			string temp_state = "blue";

			LayeredClass * temp_class = new LayeredClass(temp_current, i, temp_state);
			m_layered_map[*temp_class] = temp_lay_node;
			delete temp_class;

			m_lay_blue_node.push_back(temp_lay_node);

			if (m_verbal_flag) {
				count_blue_nodes += 1;
				cout<<"---------- Blue node: "<<count_blue_nodes<<" ----------"<<endl;
				cout<<"Current: ("<<temp_current.c_str()<<", "<<i<<", "<<temp_state.c_str()<<"), local degree = "<<temp_lay_node.loc_deg<<endl;
				for (int k = 0; k < it->loc_deg; k++) {
					cout<<"Next: (t"<<temp_lay_node.loc_neighbor_id[k]<<", "<<temp_lay_node.loc_layer[k]<<", p"
						<<boost::lexical_cast<string>(temp_lay_node.id)<<"), local weight = "<<temp_lay_node.edge_weight[k]<<endl;
				}
			}
		}
	}

	if (m_verbal_flag) {
		cout<<" "<<endl;
		cout<<"***********************************************************"<<endl;
	}

	// Target nodes
	int count_target_nodes = 0;
	for (vector<max_min_lp_msgs::general_node>::iterator it = m_gen_t_node.begin(); it != m_gen_t_node.end(); ++it) {
		for (int i = 1; i <= m_num_layer; i++) {
			for (int j = 0; j < it->loc_deg; j++) {
				max_min_lp_msgs::layered_node temp_lay_node;
				vector<int> temp_neighbor;
				for (int k = 0; k < it->loc_deg; k++) {
					temp_neighbor.push_back(it->loc_neighbor[k]);
				}
				temp_lay_node.id = it->id;
				temp_lay_node.layer = i;
				temp_lay_node.connected_id = it->loc_neighbor[j];
				temp_lay_node.state = it->type;
				temp_lay_node.g_t = 0;

				if (it->loc_deg == 1) {
					temp_lay_node.loc_deg = 1;
					temp_lay_node.loc_neighbor_id.push_back(it->loc_neighbor[j]);
					temp_lay_node.loc_layer.push_back(i);
					temp_lay_node.loc_connected_id.push_back(it->id);
					temp_lay_node.loc_state.push_back("red");
					temp_lay_node.edge_weight.push_back((it->loc_edge_weight[j])/2);
				}
				else {
					temp_neighbor.erase (temp_neighbor.begin()+j);
					temp_lay_node.loc_deg = (int)temp_neighbor.size();

					for (vector<int>::iterator itt = temp_neighbor.begin(); itt != temp_neighbor.end(); ++itt) {
						temp_lay_node.loc_neighbor_id.push_back(*itt);
						temp_lay_node.loc_layer.push_back(i);
						temp_lay_node.loc_connected_id.push_back(it->id);
						temp_lay_node.loc_state.push_back("red");

						for (int l = 0; l < it->loc_edge_weight.size(); l++) {
							if (it->loc_neighbor[l] != *itt) {
								continue;
							}
							temp_lay_node.edge_weight.push_back(it->loc_edge_weight[l]);
						}
					}
				}

				string temp_current = "t"+boost::lexical_cast<string>(temp_lay_node.id);
				string temp_state = "p"+boost::lexical_cast<string>(temp_lay_node.connected_id);

				LayeredClass * temp_class = new LayeredClass(temp_current, i, temp_state);
				m_layered_map[*temp_class] = temp_lay_node;
				delete temp_class;

				m_lay_target_node.push_back(temp_lay_node);

				if (m_verbal_flag) {
					count_target_nodes += 1;
					cout<<"---------- Target node: "<<count_target_nodes<<" ----------"<<endl;
					cout<<"Current: ("<<temp_current.c_str()<<", "<<i<<", "<<temp_state.c_str()<<"), local degree = "<<temp_lay_node.loc_deg<<endl;
					for (int k = 0; k < temp_neighbor.size(); k++) {
						cout<<"Next: (p"<<temp_lay_node.loc_neighbor_id[k]<<", "<<temp_lay_node.loc_layer[k]<<
							", red), local weight = "<<temp_lay_node.edge_weight[k]<<endl;
					}
				}
			}
		}
	}
}

void MaxMinLPCore::applyLocalAlgorithm() {
	////////////////////////////// Phase 1 //////////////////////////////
	// Set x_v with minimum of 1 / a_{r,v} for all agents which are red and blue nodes
	cout<<endl;
	cout<<"***********************************************************"<<endl;
	cout<<"Step 3 starts."<<endl;
	cout<<"***********************************************************"<<endl;
	cout<<"Phase 1 starts."<<endl;
	// Red nodes
	for (vector<max_min_lp_msgs::layered_node>::iterator it = m_lay_red_node.begin(); it != m_lay_red_node.end(); ++it) {
		float temp_max = 0;
		for (int i = 0; i < it->loc_deg; i++) {
			if (temp_max < it->edge_weight[i]) {
				temp_max = it->edge_weight[i];
			}
		}
		it->x_v = 1 / temp_max;

		map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = 
			getMapPointer("p"+boost::lexical_cast<string>(it->id), it->layer, "red");
		temp_red_pointer->second.x_v = it->x_v;

		if (m_verbal_flag) {
			cout<<"(p"<<it->id<<", "<<it->layer<<", "<<it->state.c_str()<<"), x_v = "<<it->x_v<<endl;
		}
	}

	// Blue nodes
	for (vector<max_min_lp_msgs::layered_node>::iterator it = m_lay_blue_node.begin(); it != m_lay_blue_node.end(); ++it) {
		float temp_max = 0;

		for (vector<max_min_lp_msgs::layered_node>::iterator itt = m_lay_robot_node.begin(); itt != m_lay_robot_node.end(); ++itt) {
			if (it->layer == itt->layer) {
				for (int i = 0; i < itt->loc_deg; i++) {
					if (it->id == itt->loc_neighbor_id[i]) {
						if (temp_max < itt->edge_weight[i]) {
							temp_max = itt->edge_weight[i];
						}
					}
				}
			}
		}
		it->x_v = 1 / temp_max;

		map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
			getMapPointer("p"+boost::lexical_cast<string>(it->id), it->layer, "blue");
		temp_blue_pointer->second.x_v = it->x_v;

		if (m_verbal_flag) {
			cout<<"(p"<<it->id<<", "<<it->layer<<", "<<it->state.c_str()<<"), x_v = "<<it->x_v<<endl;
		}
	}

	// Set g_t(x) for all t's
	// for (vector<max_min_lp_msgs::layered_node>::iterator it = m_lay_target_node.begin(); it != m_lay_target_node.end(); ++it) {
	// 	// Upper values (blue nodes)
	// 	map<LayeredClass, max_min_lp::layered_node>::iterator temp_pointer = 
	// 		getMapPointer("p"+boost::lexical_cast<string>(it->connected_id), it->layer-1, "blue");
	// 	for (int i = 0; i < temp_pointer->second.loc_deg; i++) {
	// 		it->g_t += temp_pointer->second.edge_weight[i] * temp_pointer->second.x_v;
	// 	}

	// 	// Lower values (red nodes)
	// 	for (int i = 0; i < it->loc_neighbor_id.size(); i++) {
	// 		map<LayeredClass, max_min_lp::layered_node>::iterator temp_pointer = 
	// 			getMapPointer("p"+boost::lexical_cast<string>(it->loc_neighbor_id[i]), it->layer, "red");
	// 		it->g_t += it->edge_weight[i] * temp_pointer->second.x_v;
	// 	}
	// }

	// Find the minimum of the values g_t(x) in S*(red) intersection with T for each agent red subset of R[0]
	m_red_tree = new TreeStruct[m_num_red_layer_zero];

	cout<<"Construct red trees (S*(r))"<<endl;
	int count_red_layer_zero = 0;
	for (vector<max_min_lp_msgs::layered_node>::iterator it = m_lay_red_node.begin(); it != m_lay_red_node.end(); ++it) {
		if (it->layer == 0) {
			getRedTreeStruct(&m_red_tree[count_red_layer_zero], "p"+boost::lexical_cast<string>(it->id), it->layer, "red");

			if (m_verbal_flag) {
				cout<<count_red_layer_zero+1<<"'s tree"<<endl;

				for (int i = 0; i < m_red_tree[count_red_layer_zero].red_node_id.size(); i++) {
					cout<<"Layer = "<<i<<endl;
					if (i > 0) {
						for (vector<string>::iterator it = m_red_tree[count_red_layer_zero].target_node_id[i-1].begin(); 
							it != m_red_tree[count_red_layer_zero].target_node_id[i-1].end(); ++it) {
							cout<<*it<<endl;
						}
					}
					for (vector<string>::iterator it = m_red_tree[count_red_layer_zero].red_node_id[i].begin(); 
						it != m_red_tree[count_red_layer_zero].red_node_id[i].end(); ++it) {
						cout<<*it<<endl;
					}
					for (vector<string>::iterator it = m_red_tree[count_red_layer_zero].robot_node_id[i].begin(); 
						it != m_red_tree[count_red_layer_zero].robot_node_id[i].end(); ++it) {
						cout<<*it<<endl;
					}
					for (vector<string>::iterator it = m_red_tree[count_red_layer_zero].blue_node_id[i].begin(); 
						it != m_red_tree[count_red_layer_zero].blue_node_id[i].end(); ++it) {
						cout<<*it<<endl;
					}
				}
			}
			count_red_layer_zero += 1;
		}
	}

	cout<<endl;
	cout<<"Find the minimum g_t(x) in S*(r) intersection with T"<<endl;

	vector<float> minimum_g_t(count_red_layer_zero, MAX_VALUE); // g_t(x)
	for (int i = 0; i < count_red_layer_zero; i++) {

		vector<string> temp_blue_node_id;
		int count_k_element = 0;
		vector<string> temp_target_pointer_verbal;

		for (int j = 0; j < m_red_tree[i].blue_node_id.size(); j++) { // Number of layers for H_0
			if (j > 0) {
				for (vector<string>::iterator it = m_red_tree[i].target_node_id[j-1].begin(); 
					it != m_red_tree[i].target_node_id[j-1].end(); ++it) {
					for (vector<string>::iterator itt = temp_blue_node_id.begin(); itt != temp_blue_node_id.end(); ++itt) {
						map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
							getMapPointer("p"+boost::lexical_cast<string>(itt->at(2)), j-1, "blue");

						for (int k = 0; k < temp_blue_pointer->second.loc_deg; k++) {
							if (temp_blue_pointer->second.loc_neighbor_id[k] == boost::lexical_cast<int>(it->at(2))) {

								float temp_g_t = 0;

								map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_target_pointer = 
									getMapPointer("t"+boost::lexical_cast<string>(it->at(2)), j, "p"+boost::lexical_cast<string>(itt->at(2)));
								temp_target_pointer_verbal.push_back("(t"+boost::lexical_cast<string>
									(it->at(2))+", "+boost::lexical_cast<string>(j)+", p"+boost::lexical_cast<string>(itt->at(2))+")");

								temp_g_t += temp_blue_pointer->second.edge_weight[k] * temp_blue_pointer->second.x_v;

								for (int l = 0; l < temp_target_pointer->second.loc_deg; l++) {
									map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = 
										getMapPointer("p"+boost::lexical_cast<string>(temp_target_pointer->second.loc_neighbor_id[l]), j, "red");

									temp_g_t += temp_target_pointer->second.edge_weight[l] * temp_red_pointer->second.x_v;
								}

								temp_target_pointer->second.g_t = temp_g_t;
								if (m_verbal_flag) {
									cout<<"(t"+boost::lexical_cast<string>(it->at(2))+", "+boost::lexical_cast<string>(j)+", p"+
										boost::lexical_cast<string>(itt->at(2))+"): g_t = "+boost::lexical_cast<string>(temp_g_t)<<endl;
								}

								// Find the minimum of g_t(x)
								if (temp_g_t < minimum_g_t.at(i)) {
									minimum_g_t.at(i) = temp_g_t;
								}

								count_k_element += 1;
							}
						}
					}
				}
			}

			temp_blue_node_id.clear();

			for (vector<string>::iterator it = m_red_tree[i].blue_node_id[j].begin(); it != m_red_tree[i].blue_node_id[j].end(); ++it) {
				if (m_verbal_flag) {
					cout<<"m_red_tree["<<i<<"].blue_node_id["<<j<<"] "<<"size() = "<<m_red_tree[i].blue_node_id[j].size()<<endl;
				}
				temp_blue_node_id.push_back(*it);
			}
		}

		set<string> temp_target_verbal_set(temp_target_pointer_verbal.begin(), temp_target_pointer_verbal.end());
		temp_target_pointer_verbal.clear();
		for (set<string>::iterator it = temp_target_verbal_set.begin(); it != temp_target_verbal_set.end(); ++it) {
			temp_target_pointer_verbal.push_back(*it);
		}

		if (m_verbal_flag) {
			cout<<"The number of elements of K is "<<temp_target_pointer_verbal.size()<<" at "<<i+1<<"'s tree"<<endl;
			for (vector<string>::iterator it = temp_target_pointer_verbal.begin(); it != temp_target_pointer_verbal.end(); ++it) {
				cout<<"Target node = "<<*it<<endl;
			}
		}
	}

	if (m_verbal_flag) {
		int temp_count = 0;
		for (vector<float>::iterator it = minimum_g_t.begin(); it != minimum_g_t.end(); ++it) {
			temp_count += 1;
			cout<<"Minimum g_t(x) at "<<temp_count<<"'s red tree is "<<*it<<endl;
		}

		int count_minimum_g_t = 0;
		for (vector<max_min_lp_msgs::layered_node>::iterator it = m_lay_red_node.begin(); it != m_lay_red_node.end(); ++it) {
			if (it->layer == 0) {
				map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = 
					getMapPointer("p"+boost::lexical_cast<string>(it->id), it->layer, "red");
				temp_red_pointer->second.t_r = minimum_g_t[count_minimum_g_t];
				count_minimum_g_t += 1;
			}
		}
	}

	cout<<"Phase 1 is done."<<endl;

	////////////////////////////// Phase 2 //////////////////////////////
	// Iteratively check if t is a valid local estimate or not until the two conditions of the paper meet. 
	//(1) t_r is a valid local estimate and 2) either (1+e)t_r is not a valid local estimate or (1+e)t_r > t_r)
	cout<<"Phase 2 starts."<<endl;
	vector<float> t_r(count_red_layer_zero); // t_r

	for (int i = 0; i < count_red_layer_zero; i++) {
		int count_recursive = 0;
		float init_minimum_g_t = minimum_g_t[i];
		t_r.at(i) = minimum_g_t[i];

		while(1) {
			count_recursive += 1;
			if (computeRecursive(i, minimum_g_t[i]) == true) {
				//if (computeRecursive(i, (1 + m_epsilon) * minimum_g_t[i]) == false) {
				//if (computeRecursive(i, m_epsilon + minimum_g_t[i]) == false) {
				t_r.at(i) = minimum_g_t[i];
				break;
				//}
				//else if ((1 + m_epsilon) * minimum_g_t[i] > init_minimum_g_t) {
				//	break;
				//}
			}

			minimum_g_t.at(i) -= m_epsilon;

			if (minimum_g_t[i] < init_minimum_g_t / 2) {
				cout<<"ERROR: t_r becomes below 1/2*t_r at "<<i+1<<"'s red tree"<<endl;
				exit (EXIT_FAILURE);
			}
		}

		if (m_verbal_flag) {
			cout<<"Number of iterations = "<<count_recursive<<endl;
			cout<<"Minimum g_t = "<<minimum_g_t[i]<<" t_r = "<<init_minimum_g_t<<" Epsilon = "<<m_epsilon<<endl;
		}
	}

	if (m_verbal_flag) {
		int temp_count = 0;
		for (vector<float>::iterator it = t_r.begin(); it != t_r.end(); ++it) {
			temp_count += 1;
			cout<<"t_r at "<<temp_count<<"'s red tree is "<<*it<<endl;
		}
	}

	cout<<"Phase 2 is done."<<endl;

	////////////////////////////// Phase 3 //////////////////////////////
	cout<<"Phase 3 starts."<<endl;

	// Each blue agent finds the minimum of the values t_r in P*(b) intersection with R[0]
	for (vector<max_min_lp_msgs::layered_node>::iterator it = m_lay_blue_node.begin(); it != m_lay_blue_node.end(); ++it) {

		vector<string> temp_target_node_id;

		for (int i = it->layer; i >= 0; i--) {
			vector<string> temp_blue_node_id;
			if (temp_target_node_id.size() != 0) {
				for (vector<string>::iterator itt = temp_target_node_id.begin(); itt != temp_target_node_id.end(); ++itt) {
					for (vector<max_min_lp_msgs::layered_node>::iterator ittt = m_lay_blue_node.begin(); ittt != m_lay_blue_node.end(); ++ittt) {
						if (ittt->layer == i) {
							for (int j = 0; j < ittt->loc_deg; j++) {
								string temp_target_node_string = "(t"+boost::lexical_cast<string>(ittt->loc_neighbor_id[j])+", "+
									boost::lexical_cast<string>(ittt->layer+1)+", p"+boost::lexical_cast<string>(ittt->id)+")";
								if (temp_target_node_string == *itt) {
									string temp_blue_node_string = "(p"+boost::lexical_cast<string>(ittt->id)+
										", "+boost::lexical_cast<string>(ittt->layer)+", blue)";
									temp_blue_node_id.push_back(temp_blue_node_string);
								}
							}
						}
					}
				}
			}
			else {
				string temp_blue_node_string = "(p"+boost::lexical_cast<string>(it->id)+
					", "+boost::lexical_cast<string>(it->layer)+", blue)";
				temp_blue_node_id.push_back(temp_blue_node_string);
			}

			if (temp_blue_node_id.size() > 0) {
				set<string> temp_blue_node_set(temp_blue_node_id.begin(), temp_blue_node_id.end());
				temp_blue_node_id.clear();
				for (set<string>::iterator itt = temp_blue_node_set.begin(); itt != temp_blue_node_set.end(); ++itt) {
					temp_blue_node_id.push_back(*itt);
				}
			}

			// Robot nodes
			vector<string> temp_robot_node_id;
			for (vector<max_min_lp_msgs::layered_node>::iterator itt = m_lay_robot_node.begin(); itt != m_lay_robot_node.end(); ++itt) {
				if (itt->layer == i) {
					for (int j = 0; j < itt->loc_deg; j++) {
						for (vector<string>::iterator ittt = temp_blue_node_id.begin(); ittt != temp_blue_node_id.end(); ++ittt) {
							if (itt->loc_neighbor_id[j] == boost::lexical_cast<int>(ittt->at(2))) {
								string temp_robot_node_string = "(r"+boost::lexical_cast<string>(itt->id)+", "+
									boost::lexical_cast<string>(itt->layer)+", p"+boost::lexical_cast<string>(itt->connected_id)+")";
								temp_robot_node_id.push_back(temp_robot_node_string);
							}
						}
					}
				}
			}

			if (temp_robot_node_id.size() > 0) {
				set<string> temp_robot_node_set(temp_robot_node_id.begin(), temp_robot_node_id.end());
				temp_robot_node_id.clear();
				for (set<string>::iterator itt = temp_robot_node_set.begin(); itt != temp_robot_node_set.end(); ++itt) {
					temp_robot_node_id.push_back(*itt);
				}
			}

			// Red nodes
			vector<string> temp_red_node_id;
			for (vector<max_min_lp_msgs::layered_node>::iterator itt = m_lay_red_node.begin(); itt != m_lay_red_node.end(); ++itt) {
				if (itt->layer == i) {
					for (int j = 0; j < itt->loc_deg; j++) {
						string temp_robot_node_string = "(r"+boost::lexical_cast<string>(itt->loc_neighbor_id[j])+", "+
							boost::lexical_cast<string>(itt->layer)+", p"+boost::lexical_cast<string>(itt->id)+")";
						for (vector<string>::iterator ittt = temp_robot_node_id.begin(); ittt != temp_robot_node_id.end(); ++ittt) {
							if (temp_robot_node_string == *ittt) {
								string temp_red_node_string = "(p"+boost::lexical_cast<string>(itt->id)+
									", "+boost::lexical_cast<string>(itt->layer)+", red)";
								temp_red_node_id.push_back(temp_red_node_string);
							}
						}
					}
				}
			}

			if (temp_red_node_id.size() > 0) {
				set<string> temp_red_node_set(temp_red_node_id.begin(), temp_red_node_id.end());
				temp_red_node_id.clear();
				for (set<string>::iterator itt = temp_red_node_set.begin(); itt != temp_red_node_set.end(); ++itt) {
					temp_red_node_id.push_back(*itt);
				}
			}

			// Target nodes
			if (i != 0) {
				for (vector<max_min_lp_msgs::layered_node>::iterator itt = m_lay_target_node.begin(); itt != m_lay_target_node.end(); ++itt) {
					if (itt->layer == i) {
						for (int j = 0; j < itt->loc_deg; j++) {
							for (vector<string>::iterator ittt = temp_red_node_id.begin(); ittt != temp_red_node_id.end(); ++ittt) {
								if (itt->loc_neighbor_id[j] == boost::lexical_cast<int>(ittt->at(2))) {
									string temp_target_node_string = "(t"+boost::lexical_cast<string>(itt->id)+", "+
										boost::lexical_cast<string>(itt->layer)+", p"+boost::lexical_cast<string>(itt->connected_id)+")";
									temp_target_node_id.push_back(temp_target_node_string);
								}
							}
						}
					}
				}

				if (temp_target_node_id.size() > 0) {
					set<string> temp_target_node_set(temp_target_node_id.begin(), temp_target_node_id.end());
					temp_target_node_id.clear();
					for (set<string>::iterator itt = temp_target_node_set.begin(); itt != temp_target_node_set.end(); ++itt) {
						temp_target_node_id.push_back(*itt);
					}
				}
			}

			// Find the minimum of the values t_r here
			if (i == 0) {
				vector<float> temp_min_t_r;
				for (vector<string>::iterator itt = temp_red_node_id.begin(); itt != temp_red_node_id.end(); ++itt) {
					temp_min_t_r.push_back(t_r[boost::lexical_cast<int>(itt->at(2)) - 1]);

					if (m_verbal_flag) {
						map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
							getMapPointer("p"+boost::lexical_cast<string>(it->id), it->layer, "blue");
						cout<<"(p"<<it->id<<", "<<it->layer<<", "<<it->state.c_str()<<"), root red node = "<<*itt<<endl;
					}
				}

				vector<float>::iterator temp_min_iterator = min_element(temp_min_t_r.begin(), temp_min_t_r.end());
				int temp_min_index = distance(temp_min_t_r.begin(), temp_min_iterator);
				it->s_b = temp_min_t_r[temp_min_index];

				map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
					getMapPointer("p"+boost::lexical_cast<string>(it->id), it->layer, "blue");
				temp_blue_pointer->second.s_b = temp_min_t_r[temp_min_index];
			}
		}

		if (m_verbal_flag) {
			map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
				getMapPointer("p"+boost::lexical_cast<string>(it->id), it->layer, "blue");
			cout<<"   minimum of the values of t_r, which is s_b = "<<temp_blue_pointer->second.s_b<<endl;
		}
	}

	// Compute recursively z(s) using equations (13)-(15) and each agent v outputs the value z_v(s)
	cout<<endl;
	cout<<"Obtain z_v's for all red and blue nodes"<<endl;
	for (int i = m_num_layer; i >= 0; i--) {
		// Blue nodes
		if (i == m_num_layer) { // Blue nodes in the last layer
			for (vector<max_min_lp_msgs::layered_node>::iterator it = m_lay_blue_node.begin(); it != m_lay_blue_node.end(); ++it) {
				if (it->layer == m_num_layer) {
					it->z_v = 0;

					if (m_verbal_flag) {
						cout<<"(p"<<it->id<<", "<<it->layer<<", blue): z_v = "<<it->z_v<<endl;
					}
				}
			}
		}
		else { // Blue nodes except the last layer
			for (vector<max_min_lp_msgs::layered_node>::iterator it = m_lay_blue_node.begin(); it != m_lay_blue_node.end(); ++it) {
				if (it->layer == i) {
					map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
						getMapPointer("p"+boost::lexical_cast<string>(it->id), it->layer, "blue");
					vector<float> temp_z_b;

					for (vector<max_min_lp_msgs::layered_node>::iterator itt = m_lay_target_node.begin(); itt != m_lay_target_node.end(); ++itt) {
						if (itt->layer == i+1) {
							for (int j = 0; j < it->loc_deg; j++) {
								string temp_next_blue_node_string = "(t"+boost::lexical_cast<string>(it->loc_neighbor_id[j])+", "+
									boost::lexical_cast<string>(it->layer+1)+", p"+boost::lexical_cast<string>(it->id)+")";
								string temp_target_node_string = "(t"+boost::lexical_cast<string>(itt->id)+", "+
									boost::lexical_cast<string>(itt->layer)+", p"+boost::lexical_cast<string>(itt->connected_id)+")";

								if (temp_next_blue_node_string == temp_target_node_string) {
									map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_target_pointer = 
										getMapPointer("t"+boost::lexical_cast<string>(itt->id), itt->layer, "p"+boost::lexical_cast<string>(it->id));
									float temp_c_k_r_z_r = 0;

									for (vector<max_min_lp_msgs::layered_node>::iterator ittt = m_lay_red_node.begin(); ittt != m_lay_red_node.end(); ++ittt) {
										if (ittt->layer == i+1) {
											for (int k = 0; k < itt->loc_deg; k++) {
												string temp_next_target_node_string = "(p"+boost::lexical_cast<string>(itt->loc_neighbor_id[k])+", "+
													boost::lexical_cast<string>(itt->layer)+", red)";
												string temp_red_node_string = "(p"+boost::lexical_cast<string>(ittt->id)+", "+
													boost::lexical_cast<string>(ittt->layer)+", red)";

												if (temp_next_target_node_string == temp_red_node_string) {
													map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = 
														getMapPointer("p"+boost::lexical_cast<string>(ittt->id), ittt->layer, "red");

													temp_c_k_r_z_r += temp_target_pointer->second.edge_weight[k] * temp_red_pointer->second.z_v;
												}
											}
										}
									}

									temp_z_b.push_back((temp_blue_pointer->second.s_b - temp_c_k_r_z_r) / temp_blue_pointer->second.edge_weight[j]);
								}
							}
						}
					}

					vector<float>::iterator temp_max_iterator = max_element(temp_z_b.begin(), temp_z_b.end());
					int temp_max_index = distance(temp_z_b.begin(), temp_max_iterator);
					temp_blue_pointer->second.z_v = max((float)0, temp_z_b[temp_max_index]);
					it->z_v = temp_blue_pointer->second.z_v;

					if (m_verbal_flag) {
						cout<<"(p"<<it->id<<", "<<it->layer<<", blue): z_v = "<<it->z_v<<endl;
					}
				}
			}
		}

		// Red nodes
		for (vector<max_min_lp_msgs::layered_node>::iterator it = m_lay_red_node.begin(); it != m_lay_red_node.end(); ++it) {
			if (it->layer == i) {
				map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = 
					getMapPointer("p"+boost::lexical_cast<string>(it->id), it->layer, "red");
				vector<float> temp_z_r;
				vector<float> debug_a_z;

				for (vector<max_min_lp_msgs::layered_node>::iterator itt = m_lay_robot_node.begin(); itt != m_lay_robot_node.end(); ++itt) {
					if (itt->layer == i) {
						for (int j = 0; j < it->loc_deg; j++) {
							string temp_next_red_node_string = "(r"+boost::lexical_cast<string>(it->loc_neighbor_id[j])+", "+
								boost::lexical_cast<string>(it->layer)+", p"+boost::lexical_cast<string>(it->id)+")";
							string temp_robot_node_string = "(r"+boost::lexical_cast<string>(itt->id)+", "+
								boost::lexical_cast<string>(itt->layer)+", p"+boost::lexical_cast<string>(itt->connected_id)+")";

							if (temp_next_red_node_string == temp_robot_node_string) {
								map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_robot_pointer = 
									getMapPointer("r"+boost::lexical_cast<string>(itt->id), itt->layer, "p"+boost::lexical_cast<string>(it->id));

								for (vector<max_min_lp_msgs::layered_node>::iterator ittt = m_lay_blue_node.begin(); ittt != m_lay_blue_node.end(); ++ittt) {
									if (ittt->layer == i) {
										for (int k = 0; k < itt->loc_deg; k++) {
											string temp_next_robot_node_string = "(p"+boost::lexical_cast<string>(itt->loc_neighbor_id[k])+", "+
												boost::lexical_cast<string>(itt->layer)+", blue)";
											string temp_blue_node_string = "(p"+boost::lexical_cast<string>(ittt->id)+", "+
												boost::lexical_cast<string>(ittt->layer)+", blue)";

											if (temp_next_robot_node_string == temp_blue_node_string) {
												map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
													getMapPointer("p"+boost::lexical_cast<string>(ittt->id), ittt->layer, "blue");

												temp_z_r.push_back((1 - temp_robot_pointer->second.edge_weight[k] * temp_blue_pointer->second.z_v) 
													/ temp_red_pointer->second.edge_weight[j]);
												debug_a_z.push_back(temp_robot_pointer->second.edge_weight[k] * temp_blue_pointer->second.z_v);
											}
										}
									}
								}
							}
						}
					}
				}

				vector<float>::iterator temp_min_iterator = min_element(temp_z_r.begin(), temp_z_r.end());
				int temp_min_index = distance(temp_z_r.begin(), temp_min_iterator);
				temp_red_pointer->second.z_v = temp_z_r[temp_min_index];
				it->z_v = temp_red_pointer->second.z_v;

				if (m_verbal_flag) {
					cout<<"(p"<<it->id<<", "<<it->layer<<", red): z_v = "<<it->z_v<<endl;
					for (vector<float>::iterator itt = debug_a_z.begin(); itt != debug_a_z.end(); ++itt) {
						cout<<"   a*z = "<<*itt<<endl;
					}
				}
			}
		}
	}

	cout<<"Phase 3 is done."<<endl;

	////////////////////////////// Step 4 //////////////////////////////
	cout<<"Step 4 starts."<<endl;

	// Map the solution of the layered max-min LP to a solution of the original max-min LP
	for (vector<max_min_lp_msgs::general_node>::iterator it = m_gen_p_r_node.begin(); it != m_gen_p_r_node.end(); ++it) {
		float temp_z_v = 0;

		// Red nodes
		for (vector<max_min_lp_msgs::layered_node>::iterator itt = m_lay_red_node.begin(); itt != m_lay_red_node.end(); ++itt) {
			if (it->id == itt->id) {
				map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = 
					getMapPointer("p"+boost::lexical_cast<string>(itt->id), itt->layer, "red");
				temp_z_v += temp_red_pointer->second.z_v;

				// if (m_verbal_flag) {
				// 	cout<<"(p"<<itt->id<<", "<<itt->layer<<", red): z_v = "<<itt->z_v<<endl;
				// }
			}
		}

		// Blue nodes
		for (vector<max_min_lp_msgs::layered_node>::iterator itt = m_lay_blue_node.begin(); itt != m_lay_blue_node.end(); ++itt) {
			if (it->id == itt->id) {
				map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
					getMapPointer("p"+boost::lexical_cast<string>(itt->id), itt->layer, "blue");
				temp_z_v += temp_blue_pointer->second.z_v;

				// if (m_verbal_flag) {
				// 	cout<<"(p"<<itt->id<<", "<<itt->layer<<", blue): z_v = "<<itt->z_v<<endl;
				// }
			}
		}

		temp_z_v = temp_z_v / (2 * (m_num_layer + 1));
		it->z_v = temp_z_v;
	}

	if (m_verbal_flag) {
		for (vector<max_min_lp_msgs::general_node>::iterator it = m_gen_p_r_node.begin(); it != m_gen_p_r_node.end(); ++it) {
			cout<<"Agent "<<it->id<<": z_v = "<<it->z_v<<endl;
		}
	}

	cout<<"Step 4 is done."<<endl;

	return m_gen_p_r_node;
}

map<LayeredClass, max_min_lp_msgs::layered_node>::iterator MaxMinLPCore::getMapPointer(string _current, int _layer, string _state) {
	LayeredClass * temp_class = new LayeredClass(_current, _layer, _state);

	if(m_layered_map.find(*temp_class) != m_layered_map.end()) {
		map<LayeredClass, max_min_lp_msgs::layered_node>::iterator return_pointer = m_layered_map.find(*temp_class);
		delete temp_class;
		return return_pointer;
	}
	else {
		cout<<"Didn't find map of ("<<_current.c_str()<<", "<<_layer<<", "<<_state.c_str()<<")"<<endl;
		delete temp_class;
		exit (EXIT_FAILURE);
	}
}

void MaxMinLPCore::getRedTreeStruct(TreeStruct * _red_tree, string _current, int _layer, string _state) {
	vector<string> temp_robot_node_id;
	vector<string> temp_red_node_id;
	vector<string> temp_blue_node_id;
	vector<string> temp_target_node_id;
	int temp_tree_depth = 0;

	// Construct the tree structure
	// Layer 0
	map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = getMapPointer(_current, _layer, _state);
	temp_red_node_id.push_back("(p"+boost::lexical_cast<string>(temp_red_pointer->second.id)+", "+
		boost::lexical_cast<string>(temp_red_pointer->second.layer)+", red)");
	_red_tree->red_node_id.push_back(temp_red_node_id);

	for (int i = 0; i < temp_red_pointer->second.loc_deg; i++) {
		map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_robot_pointer = 
			getMapPointer("r"+boost::lexical_cast<string>(temp_red_pointer->second.loc_neighbor_id[i]), 
			temp_red_pointer->second.layer, "p"+boost::lexical_cast<string>(temp_red_pointer->second.id));
		temp_robot_node_id.push_back("(r"+boost::lexical_cast<string>(temp_robot_pointer->second.id)+", "+
			boost::lexical_cast<string>(temp_robot_pointer->second.layer)+", p"+boost::lexical_cast<string>(temp_red_pointer->second.id)+")");

		for (int j = 0; j < temp_robot_pointer->second.loc_deg; j++) {
			map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
				getMapPointer("p"+boost::lexical_cast<string>(temp_robot_pointer->second.loc_neighbor_id[j]), 
				temp_robot_pointer->second.layer, "blue");
			temp_blue_node_id.push_back("(p"+boost::lexical_cast<string>(temp_blue_pointer->second.id)+", "+
				boost::lexical_cast<string>(temp_blue_pointer->second.layer)+", blue)");
		}
	}
	_red_tree->robot_node_id.push_back(temp_robot_node_id);
	_red_tree->blue_node_id.push_back(temp_blue_node_id);

	temp_robot_node_id.clear();
	temp_red_node_id.clear();
	temp_blue_node_id.clear();
	temp_target_node_id.clear();

	// Next layers
	for (int i = 1; i <= m_num_layer; i++) {
		bool check_next_layer_reach = false; // Check if this particular red tree of R[0] is able to reach the next layer. That is because it is also possible that the next layer cannot be reached.

		// Target nodes (Only target starts from index 1)
		for (vector<string>::iterator it = _red_tree->blue_node_id[i-1].begin(); it != _red_tree->blue_node_id[i-1].end(); ++it) {
			map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
				getMapPointer("p"+boost::lexical_cast<string>(it->at(2)), i-1, "blue");

			// Target nodes
			for (int j = 0; j < temp_blue_pointer->second.loc_deg; j++) {
				map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_target_pointer = 
					getMapPointer("t"+boost::lexical_cast<string>(temp_blue_pointer->second.loc_neighbor_id[j]), 
					i, "p"+boost::lexical_cast<string>(temp_blue_pointer->second.id));
				temp_target_node_id.push_back("(t"+boost::lexical_cast<string>(temp_target_pointer->second.id)+", "+
					boost::lexical_cast<string>(temp_target_pointer->second.layer)+", p"+boost::lexical_cast<string>(temp_blue_pointer->second.id)+")");

				// Red nodes
				for (int k = 0; k < temp_target_pointer->second.loc_deg; k++) {
					map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = 
						getMapPointer("p"+boost::lexical_cast<string>(temp_target_pointer->second.loc_neighbor_id[k]), 
						i, "red");
					temp_red_node_id.push_back("(p"+boost::lexical_cast<string>(temp_red_pointer->second.id)+", "+
						boost::lexical_cast<string>(temp_red_pointer->second.layer)+", red)");

					// Robot nodes
					for (int l = 0; l < temp_red_pointer->second.loc_deg; l++) {
						map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_robot_pointer = 
							getMapPointer("r"+boost::lexical_cast<string>(temp_red_pointer->second.loc_neighbor_id[l]), 
							i, "p"+boost::lexical_cast<string>(temp_red_pointer->second.id));
						temp_robot_node_id.push_back("(r"+boost::lexical_cast<string>(temp_robot_pointer->second.id)+", "+
							boost::lexical_cast<string>(temp_robot_pointer->second.layer)+", p"+
							boost::lexical_cast<string>(temp_red_pointer->second.id)+")");

						// Blue nodes
						for (int m = 0; m < temp_robot_pointer->second.loc_deg; m++) {
							map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
								getMapPointer("p"+boost::lexical_cast<string>(temp_robot_pointer->second.loc_neighbor_id[m]), 
								i, "blue");
							temp_blue_node_id.push_back("(p"+boost::lexical_cast<string>(temp_blue_pointer->second.id)+", "+
								boost::lexical_cast<string>(temp_blue_pointer->second.layer)+", blue)");

							check_next_layer_reach = true;
						}
					}
				}
			}
		}

		// Target nodes
		if (temp_target_node_id.size() > 0) {
			set<string> temp_target_node_set(temp_target_node_id.begin(), temp_target_node_id.end());
			temp_target_node_id.clear();
			for (set<string>::iterator it = temp_target_node_set.begin(); it != temp_target_node_set.end(); ++it) {
				temp_target_node_id.push_back(*it);
			}
		}
		_red_tree->target_node_id.push_back(temp_target_node_id);
		temp_target_node_id.clear();

		// Red nodes
		if (temp_red_node_id.size() > 0) {
			set<string> temp_red_node_set(temp_red_node_id.begin(), temp_red_node_id.end());
			temp_red_node_id.clear();
			for (set<string>::iterator it = temp_red_node_set.begin(); it != temp_red_node_set.end(); ++it) {
				temp_red_node_id.push_back(*it);
			}
		}
		_red_tree->red_node_id.push_back(temp_red_node_id);
		temp_red_node_id.clear();

		// Robot nodes
		if (temp_robot_node_id.size() > 0) {
			set<string> temp_robot_node_set(temp_robot_node_id.begin(), temp_robot_node_id.end());
			temp_robot_node_id.clear();
			for (set<string>::iterator it=temp_robot_node_set.begin(); it!=temp_robot_node_set.end(); ++it) {
				temp_robot_node_id.push_back(*it);
			}
		}
		_red_tree->robot_node_id.push_back(temp_robot_node_id);
		temp_robot_node_id.clear();

		// Blue nodes
		if (temp_blue_node_id.size() > 0) {
			set<string> temp_blue_node_set(temp_blue_node_id.begin(), temp_blue_node_id.end());
			temp_blue_node_id.clear();
			for (set<string>::iterator it=temp_blue_node_set.begin(); it!=temp_blue_node_set.end(); ++it) {
				temp_blue_node_id.push_back(*it);
			}
		}
		_red_tree->blue_node_id.push_back(temp_blue_node_id);
		temp_blue_node_id.clear();

		if (check_next_layer_reach) {
			temp_tree_depth += 1;
		}
	}

	_red_tree->tree_depth = temp_tree_depth;
}

bool MaxMinLPCore::computeRecursive(int _count_red_layer_zero, float _minimum_g_t) {
	bool check_z_negative = false; // True means that either z_r or z_b is negative
	for (int i = m_red_tree[_count_red_layer_zero].tree_depth; i >= 0; i--) {
		for (vector<string>::iterator it = m_red_tree[_count_red_layer_zero].blue_node_id[i].begin(); 
			it != m_red_tree[_count_red_layer_zero].blue_node_id[i].end(); ++it) {

			vector<float> temp_z_b;
			map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
				getMapPointer("p"+boost::lexical_cast<string>(it->at(2)), i, "blue");

			if (temp_blue_pointer->second.layer == i) {
				if (i == m_red_tree[_count_red_layer_zero].tree_depth) { // B[h]
					temp_blue_pointer->second.z_b = 0;
				}
				else { // B \ B[h]
					// There might be some disconnected branches in the tree. It always corresponds to blue nodes and they must be handled properly.
					if (temp_blue_pointer->second.loc_deg == 0) {
						temp_blue_pointer->second.z_b = 0;
					}
					else { // Find the maximum z_b(q)
						for (int j = 0; j < temp_blue_pointer->second.loc_deg; j++) {
							float temp_c_k_r_z_r = 0;

							map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_target_pointer = 
								getMapPointer("t"+boost::lexical_cast<string>(temp_blue_pointer->second.loc_neighbor_id[j]), 
								i+1, "p"+boost::lexical_cast<string>(it->at(2)));

							for (int k = 0; k < temp_target_pointer->second.loc_deg; k++) {
								map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = 
									getMapPointer("p"+boost::lexical_cast<string>(temp_target_pointer->second.loc_neighbor_id[k]), i+1, "red");

								temp_c_k_r_z_r += temp_target_pointer->second.edge_weight[k] * temp_red_pointer->second.z_r;
							}

							temp_z_b.push_back((_minimum_g_t - temp_c_k_r_z_r) / temp_blue_pointer->second.edge_weight[j]);
						}

						vector<float>::iterator temp_max_iterator = max_element(temp_z_b.begin(), temp_z_b.end());
						int temp_max_index = distance(temp_z_b.begin(), temp_max_iterator);
						temp_blue_pointer->second.z_b = max((float)0, temp_z_b[temp_max_index]);
					}
				}
			}
		}

		// Find the minimum z_r(q)
		for (vector<string>::iterator it = m_red_tree[_count_red_layer_zero].red_node_id[i].begin(); 
			it != m_red_tree[_count_red_layer_zero].red_node_id[i].end(); ++it) {

			vector<float> temp_z_r;
			map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_red_pointer = 
				getMapPointer("p"+boost::lexical_cast<string>(it->at(2)), i, "red");

			if (temp_red_pointer->second.layer == i) {
				for (int j = 0; j < temp_red_pointer->second.loc_deg; j++) {
					map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_robot_pointer = 
						getMapPointer("r"+boost::lexical_cast<string>(temp_red_pointer->second.loc_neighbor_id[j]), 
						i, "p"+boost::lexical_cast<string>(it->at(2)));

					for (int k = 0; k < temp_robot_pointer->second.loc_deg; k++) {
						map<LayeredClass, max_min_lp_msgs::layered_node>::iterator temp_blue_pointer = 
							getMapPointer("p"+boost::lexical_cast<string>(temp_robot_pointer->second.loc_neighbor_id[k]), i, "blue");

						temp_z_r.push_back((1 - temp_robot_pointer->second.edge_weight[k] * temp_blue_pointer->second.z_b) 
							/ temp_red_pointer->second.edge_weight[j]);
					}
				}

				vector<float>::iterator temp_min_iterator = min_element(temp_z_r.begin(), temp_z_r.end());
				int temp_min_index = distance(temp_z_r.begin(), temp_min_iterator);
				temp_red_pointer->second.z_r = temp_z_r[temp_min_index];

				if (temp_red_pointer->second.z_r < -0.0001) {
					check_z_negative = true;
					break;
				}
			}
		}

		if (check_z_negative) {
			break;
		}
	}

	// Check if all elements of z_b and z_r are positive (i.e., valid local estimate)
	if (check_z_negative) { // It is not a valid local estimate
		return false;
	}
	else { // It is a valid local estimate
		return true;
	}
}

} // namespace