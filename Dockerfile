FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle

RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY . /ros2_ws/src/ros2_navigation_project/

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select ros2_navigation_project

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run ros2_navigation_project global_planner_node"]
