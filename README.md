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

## **2. Gazebo:**
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## **3. TurtleBot3 симуляция** 

```bash
sudo apt install ros-humble-turtlebot3*
```
Установите модель робота (экспорт в каждый терминал перед запуском): 
```bash
export TURTLEBOT3_MODEL=burger
```

## **4. Опционально: slam_toolbox (для продвинутого SLAM)** 
```bash
sudo apt install ros-humble-slam-toolbox```
``` 

## **5. Опционально: Nav2 (для сравнения со стандартным навигационным стеком)** 
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

 # **Сборка проекта**

 ```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Yazan-pyth/ros2_navigation_project.git
cd ~/ros2_ws
colcon build --packages-select ros2_navigation_project
source install/setup.bash
```

## **🎮 Запуск и управление**

## **Терминал 1 – Gazebo с роботом** 
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## **Терминал 2 – узел навигации**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ros2_navigation_project global_planner_node
```

## **Терминал 3 – RViz для визуализации и управления**
```bash
ros2 run rviz2 rviz2
```

## 🖥️ Настройка RViz и отправка цели 
    1. В окне RViz в левой панели Displays:

        `Global Options → Fixed Frame = map`

    2. Добавьте необходимые отображения:

        `Add → Map – показывает строящуюся карту (топик /map).`

        `Add → Path – выберите топик /global_plan – показывает траекторию, найденную A*.`

    3. Выберите инструмент 2D Goal Pose (кнопка на верхней панели).
    4. Кликните левой кнопкой мыши на карте в том месте, куда нужно отправить робота.
    При клике RViz публикует сообщение в топик `/goal_pose`. Ваш узел получит цель, построит A*-путь, и робот начнёт движение, отображая путь в RViz.
