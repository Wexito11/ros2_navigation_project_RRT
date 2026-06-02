FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle
ENV ROS_DOMAIN_ID=0

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy source and build
WORKDIR /ros2_ws
COPY . /ros2_ws/src/ros2_navigation_project/

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select ros2_navigation_project && \
    echo 'Build successful'"

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
