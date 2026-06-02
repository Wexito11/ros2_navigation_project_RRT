#!/bin/bash
echo "=== ROS2 RRT Navigation ==="
echo "Allowing GUI access..."
xhost +local:docker

echo "Starting simulation and navigation..."
docker-compose up --build
