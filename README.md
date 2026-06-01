# ROS2 Navigation — RRT Global Planner

МГТУ им. Н.Э. Баумана, группа СМ7и-64Б
Студент: Уэкслер Д.С. | Преподаватель: Вассуф Я.

## Запуск

Терминал 1:
```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Терминал 2:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_navigation_project global_planner_node
```

Терминал 3:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2
```
