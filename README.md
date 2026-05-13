# ROS2 Navigation Project: Global A* Planner + Pure Pursuit Controller

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**Проект** демонстрирует автономную навигацию мобильного робота (TurtleBot3) в симуляторе Gazebo с использованием ROS2 Humble. Реализовано:

- построение карты (`/map`) в реальном времени на основе данных лидара (`/scan`);
- глобальное планирование пути с помощью алгоритма A*;
- локальное управление методом Pure Pursuit;
- визуализация в RViz, включая установку цели через `2D Goal Pose`.

## 📌 Репозиторий

Склонируйте проект:

```bash
git clone https://github.com/Yazan-pyth/ros2_navigation_project.git
```
## **Установка необходимых инструментов**
Перед началом работы убедитесь, что у вас установлены следующие компоненты: 

## **1. Проверьте установку РОС2:** 
```bash
ros2 --version
echo $ROS_DISTRO   # должно быть humble 
```

## ** 2. Gazebo:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## ** 3. TurtleBot3 симуляция 

```bash
sudo apt install ros-humble-turtlebot3*
```
Установите модель робота (экспорт в каждый терминал перед запуском): 
```bash
export TURTLEBOT3_MODEL=burger
```

## ** 4. Опционально: slam_toolbox (для продвинутого SLAM) 
```bash
sudo apt install ros-humble-slam-toolbox```
``` 

## ** 5. Опционально: Nav2 (для сравнения со стандартным навигационным стеком) 
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

 # `Сборка проекта`

 ```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Yazan-pyth/ros2_navigation_project.git
cd ~/ros2_ws
colcon build --packages-select ros2_navigation_project
source install/setup.bash
```

## **🎮 Запуск и управление**
