FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]

ARG USER_NAME
ARG BACKEND_URL
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USER_NAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USER_NAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USER_NAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER_NAME \
    && chmod 0440 /etc/sudoers.d/$USER_NAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

# Enter user
USER $USER_NAME
WORKDIR /home/$USER_NAME

# Install ROS 2 dev tools
RUN sudo apt update -y
RUN sudo apt upgrade -y
RUN sudo apt install ros-dev-tools -y

# Set up mir_robot package
RUN mkdir -p ~/ros2_ws/src/mir_data_collection
RUN cd ~/ros2_ws/ && git clone -b humble-devel https://github.com/relffok/mir_robot src/mir_robot
RUN cd ~/ros2_ws/ && vcs import < src/mir_robot/ros2.repos src --recursive
RUN sudo apt update -y
RUN sudo apt upgrade -y
RUN sudo apt install -y python3-rosdep
RUN rosdep update -y 
RUN rosdep update --rosdistro=humble -y
RUN cd ~/ros2_ws/ && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Build and set source
RUN cd ~/ros2_ws/ && source /opt/ros/humble/setup.bash && colcon build
RUN cd ~/ros2_ws/ && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN cd ~/ros2_ws/ && sudo echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Copy directory into container
COPY . /home/$USER_NAME/ros2_ws/src/mir_data_collection

# ROS_py dependencies
RUN sudo apt install ros-humble-rosbag2 -y && sudo apt install ros-humble-rosbag2-storage-mcap -y && sudo apt install ros-humble-rosbag2-py -y && sudo apt install ros-humble-rclpy -y && sudo apt-get install python3-requests -y

RUN sudo chmod 777 -R ~/ros2_ws/

CMD bash /home/${USER_NAME}/ros2_ws/src/mir_data_collection/mir_data_collection/start_data_collection.sh
