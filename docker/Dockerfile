FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    git \
    ros-humble-moveit \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-servo \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit-resources-panda-description \
    && rm -rf /var/lib/apt/lists/*

# Install MediaPipe and Python dependencies
COPY /requirements.txt /tmp/requirements.txt
RUN pip3 install --upgrade pip && pip3 install -r /tmp/requirements.txt

# Create workspace
RUN mkdir -p /ws/src
WORKDIR /ws


# Copy the whole workspace (preserve src/ structure)
COPY . /ws/

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Source ROS and workspace on container start
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && bash"]