# multi-robot_task_allocation
Algorithms of a multi-robot task allocation for multi-target tracking are provided, which have been developed in ROS Kinetic on Ubuntu 16.04. This package provides several features for a multi-robot task allocation when tracking multiple targets. A local algorithm plays a key role in this algorithm where it provides nice properties on communication limitation and was referred by Floréen, Patrik, et al. "Local approximability of max-min and min-max linear programs." Theory of Computing Systems 49.4 (2011): 672-697.

## Run a demo

Demo is included in max_min_lp_demo. In max_min_lp_demo/data there are several instances that you can test. Local algorithm is implemented in a sequential way, not in a distributed since one machine is used to test the performance of the algorithm.

```
roslaunch max_min_lp_demo test_max_min_lp.launch
```
In case you want to visualize an original and layered graphs of instance that you are testing, run the following.

```
rosrun max_min_lp_visualization max_min_lp_plot.py
```

Then, the result of the graph will be in the max_min_lp_visualization/log folder. The following will activate the demo.

```
rostopic pub /robot_status std_msgs/String demo
```

## Two main algorithms

There are two versions of multi-robot multi-target tracking algorithms. The first algorithm is a local algorithm based one, which is explained in the beginning. The second algorithm is a greedy one where each robot greedily track targets without considering other robots. For both a central node exists to simulate sensing and communication graphs. Each robot has its own node for local computation and getting odometry information from the gazebo simulation. 

### Local algorithm

Run the gazebo simulation first. This will spawn five (turtlebot) robots with thirty stationary and moving targets.

```
roslaunch max_min_lp_simulation five_robot_twenty_target_case.launch
```
Then, run the main algorithm.

```
roslaunch max_min_lp_simulation simulation_server_to_robots.launch
```

To initiate the algorithm,

```
rostopic pub /robot_status std_msgs/String run
```


### Greedy algorithm

Again, run the gazebo simulation first.

```
roslaunch max_min_lp_simulation five_robot_twenty_target_case.launch
```
Then, run the main algorithm.

```
roslaunch max_min_lp_simulation greedy_server_to_robots.launch
```

To initiate the algorithm,
```
rostopic pub /robot_status std_msgs/String run
```


## Generating and testing motion primitives of a robot

In this package, which is max_min_lp_simulation, turtlebot is used as a mobile platform. This feature allows to generate and test different sets of motion primitives of a robot while changing `cmd_vel` values with different time intervals. In our setting time interval is defined by the number of publishers applied to. First, run the following command.

```
roslaunch max_min_lp_simulation single_robot_case.launch
```

This will run the gazebo simulator with a single robot.

```
rosrun max_min_lp_simulation motion_primitive_gerator
```

Now you can feed `/robot/status` rostopic in order to move the robot. In the following the first command moves the robot `x direction` and the second one rotates the robot in the counter-clockwise. In the gazebo simulator the positive values for `x` is the forward direction to the robot while the positive values for `z` rotates the robot in the counter-clockwise.

```
rostopic pub /robot_status std_msgs/String x
rostopic pub /robot_status std_msgs/String z
```

You can also check the odometry information of the robot by using `rostopic echo /robot_1/odom`.
