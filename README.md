Trajectory Tracking Controller for TurtleBot3 (ROS 2 Humble)

This repository contains the solution for implementing a path smoothing and trajectory tracking controller for a differential drive robot (TurtleBot3) in a simulated 2D environment using ROS 2 Humble.

Objective
The primary goal is to take a coarse set of waypoints, generate a smooth, dense path using two different smoothing algorithms, and then implement a simple Proportional (P) controller to guide the TurtleBot3 along this generated trajectory using odometry feedback.

System Requirements & Setup
Environment used for development: - Operating System: Ubuntu 22.04 LTS - ROS 2 Distribution: Humble Hawksbill - Simulator: Gazebo Classic (Version 11) - Robot Platform: TurtleBot3 (Waffle)

    Installation

Ensure you have the necessary ROS 2 packages installed.
1.1 Install core ROS 2 packages

1) sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

1.2 Install TurtleBot3 simulation and base packages

2) sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3*

1.3 install matlab and numpy

     Workspace Setup

Follow these steps:

3) cd ~/10xconstuction_ws/

4) colcon build --packages-select tb_controller

5) source install/setup.bash

6) export TURTLEBOT3_MODEL=waffle

     Execution Instructions

3.0 locate to src/tb_controller/trajectory and run test_trajectory.py to check the smoothing algorithm in action. One is custom smoothing algorithm and another is library based.

The system runs using three ROS 2 nodes.
3.1 Launch the Gazebo Simulation and RViz
Launch Gazebo

7) ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Launch RViz with Navigation2

8) ros2 launch turtlebot3_navigation2 navigation2.launch.py \
      use_sim_time:=True \
      map:=/home/nobel/10xconstructions/src/tb_controller/maps/nav_map.yaml

3.2 RViz Configuration Steps
    1. Set Initial Robot Pose: Use the 2D Pose Estimate tool.
    2. Visualize the Path:
        ◦ Add → rviz_default_plugins → Path
        ◦ Set topic to:
        ◦ /custom_trajectory

3.3 Start the Trajectory Publisher (Path Generation)
This node loads waypoints, smooths them, and publishes the path.

9) ros2 run tb_controller trajectory_publisher

3.4 Start the Trajectory Controller (Execution)
This node subscribes to /custom_trajectory and /odom and publishes /cmd_vel.

10) ros2 run tb_controller trajectory_controller

4. Extending to a Real Robot

To deploy this on real hardware: 
1. Subscribe to physical robot odometry. 
2. Generate smooth path using trajectory classes. 
3. Use current pose + goal pose to compute appropriate cmd_vel.
