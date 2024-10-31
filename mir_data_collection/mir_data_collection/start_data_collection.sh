#!/bin/bash

# Source
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Trap to kill background processes on exit or interruption
trap "kill $gazebo_pid $amcl_pid $navigation_pid" EXIT INT

# Start Gazebo simulation in the background
ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz &
gazebo_pid=$!

# Start Adaptive Monte Carlo localization
ros2 launch mir_navigation amcl.py use_sim_time:=true map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml &
amcl_pid=$!

# Start navigation
ros2 launch mir_navigation navigation.py use_sim_time:=true &
navigation_pid=$!

sleep 7

# Set initial robot pose for navigation to work
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'

# Continuously send random goal positions
while true; do
  # Generate random floating-point x and y between -5 and 5
  x=$(awk -v min=-5 -v max=5 'BEGIN{srand(); print min+rand()*(max-min)}')
  y=$(awk -v min=-5 -v max=5 'BEGIN{srand(); print min+rand()*(max-min)}')

  # Send goal position to the robot
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{
  header: {
    frame_id: 'map'
  },
  pose: {
    position: {x: $x, y: $y, z: 0.0},
    orientation: {z: 0.0, w: 1.0}
  }
}"

  # Wait 10 seconds before sending another goal
  sleep 10
done

# Wait for all background processes to finish
wait



