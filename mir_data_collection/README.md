### Random useful notes

# Install their repository following their README
https://github.com/relffok/mir_robot

# Ros humble setup
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-humble-rosbag2
sudo apt install ros-humble-rosbag2-storage-mcap

# This package setup
cd ~/ros2_ws/

git clone -b main https://github.com/idinyte/ros_data_collection_SDU src/mir_data_collection

# Run simulation (execute all three in different terminals)
ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz

ros2 launch mir_navigation amcl.py use_sim_time:=true map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml

ros2 launch mir_navigation navigation.py use_sim_time:=true

# Set initial pose (in fourth terminal, otherwise navigation doesnt work)
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'

# Set goal pose
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{
  header: {
    frame_id: "map"
  },
  pose: {
    position: {x: -2.0, y: 0.0, z: 0.0},
    orientation: {z: 0.0, w: 1.0}
  }
}'

# Goal pose orientation from euler rotation to orientation queternion
yaw = math.radians(90)  # Convert degrees to radians

z = math.sin(yaw / 2)

w = math.cos(yaw / 2)

# Build our data collection package (every time you modify it)
cd ~/ros2_ws/

colcon build --packages-select mir_data_collection

source install/setup.bash

# Run our package (another terminal...)
ros2 run mir_data_collection sensor_monitor
