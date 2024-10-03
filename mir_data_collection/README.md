### Random useful notes

# ## Create new package
# cd ~/ros2_ws/src/
# ros2 pkg create --build-type ament_python mir_data_collection --dependencies rclpy sensor_msgs nav_msgs

# # Add sensor monitor script
# cd mir_data_collection/mir_data_collection
# nano sensor_monitor.py

# # Update package configuration
# cd ../
# nano setup.py

# Install as in git instructions from
https://github.com/relffok/mir_robot

# Extra setup
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-humble-rosbag2 # not sure if necessary actually

# Setup this package
cd ~/ros2_ws/
git clone -b main https://github.com/idinyte/ros_data_collection_SDU src/mir_data_collection

# Run simulation
ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz
ros2 launch mir_navigation amcl.py use_sim_time:=true map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml
ros2 launch mir_navigation navigation.py use_sim_time:=true

# Set initial pose (otherwise navigation doesnt work)
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'

# Set goal pose
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{
  header: {
    frame_id: "map"
  },
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {z: 0.0, w: 1.0}
  }
}'

# Goal pose orientation from euler rotation to orientation queternion
yaw = math.radians(90)  # Convert degrees to radians
z = math.sin(yaw / 2)
w = math.cos(yaw / 2)

# Build our package
cd ~/ros2_ws/
colcon build --packages-select mir_data_collection
source install/setup.bash

# Run our package
ros2 run mir_data_collection sensor_monitor
