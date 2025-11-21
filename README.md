Trajectory Tracking Controller for TurtleBot3 (ROS 2 Humble)

This repository contains the solution for implementing a path smoothing and trajectory tracking controller for a differential drive robot (TurtleBot3) in a simulated 2D environment using ROS 2 Humble.

Objective

The primary goal is to take a coarse set of waypoints, generate a smooth, dense path using two different smoothing algorithms, and then implement a simple Proportional (P) controller to guide the TurtleBot3 along this generated trajectory using odometry feedback.

System Requirements & Setup

This project was developed and tested in the following environment:

Operating System: Ubuntu 22.04 LTS
ROS 2 Distribution: Humble Hawksbill
Simulator: Gazebo Classic (Version 11)
Robot Platform: TurtleBot3 (Waffle)


    1. Installation

Ensure you have the necessary ROS 2 packages installed:
# Core ROS 2 packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup 
# TurtleBot3 simulation and base packages
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3*


    2. Workspace Setup


# 1. Navigate to your workspace root
cd ~/10xconstuction_ws/
# 2. Build the package
colcon build --packages-select tb_controller
# 3. Source the setup files (MANDATORY for every new terminal)
source install/setup.bash
# 4. Set the TurtleBot3 model environment variable
export TURTLEBOT3_MODEL=waffle 


    3. Execution Instructions
The system is launched using three independent nodes that communicate via standard ROS topics.

    Step 3.1: Launch the Gazebo Simulation and Rviz
# 1. Launch the Gazebo environment 
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
# 2. Launch Rviz and the Navigation2 stack with the specified map
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=True \
    map:=/home/nobel/10xconstructions/src/tb_controller/maps/nav_map.yaml


    Step 3.2: Configure RViz for Visualization and Initial Pose
After launching the simulation and RViz, perform these manual steps in the RViz window:
Set Initial Robot Pose: Click the "2D Pose Estimate" button (often looks like a robot icon or an arrow) in the RViz toolbar. Click on the map where you want the robot to start, and drag to set the initial orientation. This publishes a pose to the /initialpose topic, aligning the robot's internal pose estimate with the physical robot's starting position.

Visualize the Path:
Click the "Add" button in the bottom left panel (Displays).
In the pop-up window, select the rviz_default_plugins entry, and choose the Path display type. Click "OK".
In the newly added Path display configuration, change the Topic field from the default to /custom_trajectory.
Once the trajectory_publisher starts, the smoothed path will appear in RViz, allowing you to confirm the path being sent to the controller.


    Step 3.3: Start the Trajectory Publisher (Path Generation)
This node publishes the planned, smoothed path to the controller.
Function: Loads waypoints, applies smoothing, and publishes the resulting path as a stream of nav_msgs/Path.


Smoothing Algorithms:
Custom Algorithm: A custom implementation for basic path refinement.
Cubic Spline Interpolation: Utilizes scipy.interpolate.CubicSpline for mathematically continuous and smooth paths.

ros2 run tb_controller trajectory_publisher 


    Step 3.4: Start the Trajectory Controller (Execution)

This is the node containing the P-Controller logic that subscribes to the path and odometry, and issues velocity commands.

ros2 run tb_controller trajectory_controller 


  
## 5. Extending to a Real Robot

Transitioning this solution from simulation to a physical TurtleBot3 requires addressing several key differences:
1) we will susbcribe to odometry topic of robot and get position
or if any other kind of positioning coordinate is available well subscribe that.
2) will generate path using the trajectory building classes
3) with with current pose and goal pose, we can generate adequate cmd velocity for the robot to follow the trajectory.

