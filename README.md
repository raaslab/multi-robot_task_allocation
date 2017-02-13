# multi-robot_task_allocation
Algorithm of a multi-robot task allocation for multi-target tracking

This package provides several features for a multi-robot task allocation when tracking multiple targets.

1. Generating and testing motion primitives of a robot

In this package turtlebot is used as a mobile platform. This feature allows to generate and test different sets of motion primitives of a robot while changing `cmd_vel` values with different time intervals. In our setting time interval is defined by the number of publishers applied to. First, run the following command.

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
