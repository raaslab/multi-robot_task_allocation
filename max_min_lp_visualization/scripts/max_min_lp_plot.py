#!/usr/bin/env python

'''
  Visualize the result of max_min_lp package.
  \Author Yoonchang Sung <yooncs8@vt.edu>
  \12/03/2016
  Copyright 2016. All Rights Reserved.
'''

import rospy
import rospkg
from std_msgs.msg import String
from max_min_lp_msgs.msg import general_node_array
from max_min_lp_msgs.msg import layered_node_array
import pygraphviz as pg

def general_callback(data):
    AG = pg.AGraph(directed=False, strict=True)
    AG.layout(prog='dot')

    rospy.loginfo("General node reading starts..")

    count_robot = 0
    count_primitive = 0
    count_target = 0

    for gen_node_id in data.gen_nodes:
        if gen_node_id.type == "robot":
            count_robot = count_robot + 1
        if gen_node_id.type == "red":
            count_primitive = count_primitive + 1
        if gen_node_id.type == "target":
            count_target = count_target + 1

    rospy.loginfo("Number of robot general nodes = %d", count_robot)
    rospy.loginfo("Number of primitive general nodes = %d", count_primitive)
    rospy.loginfo("Number of target general nodes = %d", count_target)

    interval_nodes = 150

    if count_robot%2 == 0:
        pos_robot_start = -1*(count_robot/2-1)*interval_nodes-interval_nodes/2
    else:
        pos_robot_start = -1*(count_robot-1)/2*interval_nodes
    if count_primitive%2 == 0:
        pos_primitive_start = -1*(count_primitive/2-1)*interval_nodes-interval_nodes/2
    else:
        pos_primitive_start = -1*(count_primitive-1)/2*interval_nodes
    if count_target%2 == 0:
        pos_target_start = -1*(count_target/2-1)*interval_nodes-interval_nodes/2
    else:
        pos_target_start = -1*(count_target-1)/2*interval_nodes

    inc_robot = 0
    inc_primitive = 0
    inc_target = 0

    # Nodes for general graph
    for gen_node_id in data.gen_nodes:
        if gen_node_id.type == "robot":
            temp_node_name = gen_node_id.type+str(gen_node_id.id)
            AG.add_node(temp_node_name)
            n = AG.get_node(temp_node_name)
            n.attr['pos'] = "%f,%f)"%(pos_robot_start+inc_robot*interval_nodes,75)
            inc_robot += 1
        if gen_node_id.type == "red":
            temp_node_name = "primitive"+str(gen_node_id.id)
            AG.add_node(temp_node_name)
            n = AG.get_node(temp_node_name)
            n.attr['pos'] = "%f,%f)"%(pos_primitive_start+inc_primitive*interval_nodes,0)
            inc_primitive += 1
        if gen_node_id.type == "target":
            temp_node_name = gen_node_id.type+str(gen_node_id.id)
            AG.add_node(temp_node_name)
            n = AG.get_node(temp_node_name)
            n.attr['pos'] = "%f,%f)"%(pos_target_start+inc_target*interval_nodes,-75)
            inc_target += 1

    # Edges for general graph
    for gen_node_id in data.gen_nodes:
        if gen_node_id.type == "robot":
            count_robot_loc = 0;
            for loc_neighbor in gen_node_id.loc_neighbor:
                temp_node_name = "robot"+str(gen_node_id.id)
                temp_neighbor_node_name = "primitive"+str(loc_neighbor)
                AG.add_edge(temp_node_name, temp_neighbor_node_name)
                e = AG.get_edge(temp_node_name, temp_neighbor_node_name)
                e.attr['weight'] = gen_node_id.loc_edge_weight[count_robot_loc]
                temp_weight = gen_node_id.loc_edge_weight[count_robot_loc]
                temp_weight = float("{0:.2f}".format(temp_weight))
                e.attr['label'] = str(temp_weight)
                count_robot_loc += 1
        if gen_node_id.type == "target":
            count_target_loc = 0;
            for loc_neighbor in gen_node_id.loc_neighbor:
                temp_node_name = "target"+str(gen_node_id.id)
                temp_neighbor_node_name = "primitive"+str(loc_neighbor)
                AG.add_edge(temp_node_name, temp_neighbor_node_name)
                e = AG.get_edge(temp_node_name, temp_neighbor_node_name)
                e.attr['weight'] = gen_node_id.loc_edge_weight[count_target_loc]
                temp_weight = gen_node_id.loc_edge_weight[count_target_loc]
                temp_weight = float("{0:.2f}".format(temp_weight))
                e.attr['label'] = str(temp_weight)
                count_target_loc += 1

    rospack = rospkg.RosPack()
    AG.write(rospack.get_path('max_min_lp_visualization')+"/log/general_graph.dot")
    AG.draw(rospack.get_path('max_min_lp_visualization')+"/log/general_graph.png",prog='neato',args='-n2')

def layered_callback(data):
    LG = pg.AGraph(directed=False, strict=True)
    LG.layout(prog='dot')

    rospy.loginfo("layered nodes reading starts..")

    max_layer = 0

    for lay_node_id in data.lay_nodes:
        if lay_node_id.state == "red":
            if lay_node_id.layer > max_layer:
                max_layer = lay_node_id.layer

    rospy.loginfo("Maximum layer = %d", max_layer)

    count_robot = []
    count_red = []
    count_blue = []
    count_target = []

    for i in range(0,max_layer+1):
        count_robot.append(0)
        count_red.append(0)
        count_blue.append(0)
    for i in range(max_layer):
        count_target.append(0)

    for lay_node_id in data.lay_nodes:
        if lay_node_id.state == "robot":
            for i in range(0, max_layer+1):
                if lay_node_id.layer == i:
                    count_robot[i] += 1
        if lay_node_id.state == "red":
            for i in range(0, max_layer+1):
                if lay_node_id.layer == i:
                    count_red[i] += 1
        if lay_node_id.state == "blue":
            for i in range(0, max_layer+1):
                if lay_node_id.layer == i:
                    count_blue[i] += 1
        if lay_node_id.state == "target":
            for i in range(0, max_layer):
                if lay_node_id.layer-1 == i:
                    count_target[i] += 1

    for i in range(0, max_layer+1):
        rospy.loginfo("Number of robot layered nodes = %d at layer %d", count_robot[i], i)
        rospy.loginfo("Number of red layered nodes = %d at layer %d", count_red[i], i)
        rospy.loginfo("Number of blue layered nodes = %d at layer %d", count_blue[i], i)
    for i in range(0, max_layer):
        rospy.loginfo("Number of target layered nodes = %d at layer %d", count_target[i], i+1)

    interval_nodes = 150

    pos_robot_start = []
    pos_red_start = []
    pos_blue_start = []
    pos_target_start = []

    for i in range(0, max_layer+1):
        if count_robot[i]%2 == 0:
            pos_robot_start.append(-1*(count_robot[i]/2-1)*interval_nodes-interval_nodes/2)
        else:
            pos_robot_start.append(-1*(count_robot[i]-1)/2*interval_nodes)
        if count_red[i]%2 == 0:
            pos_red_start.append(-1*(count_red[i]/2-1)*interval_nodes-interval_nodes/2)
        else:
            pos_red_start.append(-1*(count_red[i]-1)/2*interval_nodes)
        if count_blue[i]%2 == 0:
            pos_blue_start.append(-1*(count_blue[i]/2-1)*interval_nodes-interval_nodes/2)
        else:
            pos_blue_start.append(-1*(count_blue[i]-1)/2*interval_nodes)
    for i in range(0, max_layer):
        if count_target[i]%2 == 0:
            pos_target_start.append(-1*(count_target[i]/2-1)*interval_nodes-interval_nodes/2)
        else:
            pos_target_start.append(-1*(count_target[i]-1)/2*interval_nodes)

    inc_robot = []
    inc_red = []
    inc_blue = []
    inc_target = []

    for i in range(0,max_layer+1):
        inc_robot.append(0)
        inc_red.append(0)
        inc_blue.append(0)
    for i in range(max_layer):
        inc_target.append(0)

    init_red_start = 0
    init_robot_start = -75
    init_blue_start = -150
    init_target_start = -225

    # Nodes for layered graph
    for lay_node_id in data.lay_nodes:
        for i in range(0, max_layer+1):
            if lay_node_id.state == "robot":
                if lay_node_id.layer == i:
                    temp_node_name = "(r"+str(lay_node_id.id)+","+str(lay_node_id.layer)+",p"+str(lay_node_id.connected_id)+")"
                    LG.add_node(temp_node_name)
                    n = LG.get_node(temp_node_name)
                    n.attr['pos'] = "%f,%f)"%(pos_robot_start[i]+inc_robot[i]*interval_nodes,init_robot_start-300*i)
                    rospy.loginfo("(r"+str(lay_node_id.id)+",%d) pos=(%f,%f)",lay_node_id.layer, 
                        pos_robot_start[i]+inc_robot[i]*interval_nodes,init_robot_start-300*i)
                    inc_robot[i] += 1
            if lay_node_id.state == "red":
                if lay_node_id.layer == i:
                    temp_node_name = "(p"+str(lay_node_id.id)+","+str(lay_node_id.layer)+",red)"
                    LG.add_node(temp_node_name)
                    n = LG.get_node(temp_node_name)
                    n.attr['pos'] = "%f,%f)"%(pos_red_start[i]+inc_red[i]*interval_nodes,init_red_start-300*i)
                    rospy.loginfo("(red"+str(lay_node_id.id)+",%d) pos=(%f,%f)",lay_node_id.layer, 
                        pos_red_start[i]+inc_red[i]*interval_nodes,init_red_start-300*i)
                    inc_red[i] += 1
            if lay_node_id.state == "blue":
                if lay_node_id.layer == i:
                    temp_node_name = "(p"+str(lay_node_id.id)+","+str(lay_node_id.layer)+",blue)"
                    LG.add_node(temp_node_name)
                    n = LG.get_node(temp_node_name)
                    n.attr['pos'] = "%f,%f)"%(pos_blue_start[i]+inc_blue[i]*interval_nodes,init_blue_start-300*i)
                    rospy.loginfo("(blue"+str(lay_node_id.id)+",%d) pos=(%f,%f)",lay_node_id.layer, 
                        pos_blue_start[i]+inc_blue[i]*interval_nodes,init_blue_start-300*i)
                    inc_blue[i] += 1
            if i > 0:
                if lay_node_id.state == "target":
                    if lay_node_id.layer == i:
                        temp_node_name = "(t"+str(lay_node_id.id)+","+str(lay_node_id.layer)+",p"+str(lay_node_id.connected_id)+")"
                        LG.add_node(temp_node_name)
                        n = LG.get_node(temp_node_name)
                        n.attr['pos'] = "%f,%f)"%(pos_target_start[i-1]+inc_target[i-1]*interval_nodes,init_target_start-300*(i-1))
                        rospy.loginfo("(target"+str(lay_node_id.id)+",%d) pos=(%f,%f)",lay_node_id.layer, 
                            pos_target_start[i-1]+inc_target[i-1]*interval_nodes,init_target_start-300*(i-1))
                        inc_target[i-1] += 1

    # Edges for general graph
    for lay_node_id in data.lay_nodes:
        for i in range(0, max_layer+1):
            if lay_node_id.state == "robot":
                if lay_node_id.layer == i:
                    for j in range(0, lay_node_id.loc_deg):
                        temp_node_name = "(r"+str(lay_node_id.id)+","+str(lay_node_id.layer)+",p"+str(lay_node_id.connected_id)+")"
                        temp_node_neighbor_name = "(p"+str(lay_node_id.loc_neighbor_id[j])+","+str(lay_node_id.layer)+",blue)"
                        LG.add_edge(temp_node_name, temp_node_neighbor_name)
                        e = LG.get_edge(temp_node_name, temp_node_neighbor_name)
                        e.attr['weight'] = lay_node_id.edge_weight[j]
                        temp_weight = lay_node_id.edge_weight[j]
                        temp_weight = float("{0:.2f}".format(temp_weight))
                        e.attr['label'] = str(temp_weight)
            if lay_node_id.state == "red":
                if lay_node_id.layer == i:
                    for j in range(0, lay_node_id.loc_deg):
                        temp_node_name = "(p"+str(lay_node_id.id)+","+str(lay_node_id.layer)+",red)"
                        temp_node_neighbor_name = "(r"+str(lay_node_id.loc_neighbor_id[j])+","+str(lay_node_id.layer)+",p"+str(lay_node_id.id)+")"
                        LG.add_edge(temp_node_name, temp_node_neighbor_name)
                        e = LG.get_edge(temp_node_name, temp_node_neighbor_name)
                        e.attr['weight'] = lay_node_id.edge_weight[j]
                        temp_weight = lay_node_id.edge_weight[j]
                        temp_weight = float("{0:.2f}".format(temp_weight))
                        e.attr['label'] = str(temp_weight)
            if i < max_layer:
                if lay_node_id.state == "blue":
                    if lay_node_id.layer == i:
                        for j in range(0, lay_node_id.loc_deg):
                            temp_node_name = "(p"+str(lay_node_id.id)+","+str(lay_node_id.layer)+",blue)"
                            temp_node_neighbor_name = "(t"+str(lay_node_id.loc_neighbor_id[j])+","+str(lay_node_id.layer+1)+",p"+str(lay_node_id.id)+")"
                            LG.add_edge(temp_node_name, temp_node_neighbor_name)
                            e = LG.get_edge(temp_node_name, temp_node_neighbor_name)
                            e.attr['weight'] = lay_node_id.edge_weight[j]
                            temp_weight = lay_node_id.edge_weight[j]
                            temp_weight = float("{0:.2f}".format(temp_weight))
                            e.attr['label'] = str(temp_weight)
            if i > 0:
                if lay_node_id.state == "target":
                    if lay_node_id.layer == i:
                        for j in range(0, lay_node_id.loc_deg):
                            temp_node_name = "(t"+str(lay_node_id.id)+","+str(lay_node_id.layer)+",p"+str(lay_node_id.connected_id)+")"
                            temp_node_neighbor_name = "(p"+str(lay_node_id.loc_neighbor_id[j])+","+str(lay_node_id.layer)+",red)"
                            LG.add_edge(temp_node_name, temp_node_neighbor_name)
                            e = LG.get_edge(temp_node_name, temp_node_neighbor_name)
                            e.attr['weight'] = lay_node_id.edge_weight[j]
                            temp_weight = lay_node_id.edge_weight[j]
                            temp_weight = float("{0:.2f}".format(temp_weight))
                            e.attr['label'] = str(temp_weight)

    rospack = rospkg.RosPack()
    LG.write(rospack.get_path('max_min_lp_visualization')+"/log/layered_graph.dot")
    LG.draw(rospack.get_path('max_min_lp_visualization')+"/log/layered_graph.png",prog='neato',args='-n2')
    
def MaxMinLPVisualization():
    rospy.init_node('MaxMinLPVisualization', anonymous=True)
    rospy.Subscriber("max_min_lp_msgs/general_node_array", general_node_array, general_callback)
    rospy.Subscriber("max_min_lp_msgs/layered_node_array", layered_node_array, layered_callback)
    rospy.spin()

if __name__ == '__main__':
    MaxMinLPVisualization()