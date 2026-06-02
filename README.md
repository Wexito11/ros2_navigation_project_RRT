# ROS2 Navigation — RRT Global Planner

> Курсовой проект по дисциплине **«Программное обеспечение управляющих комплексов»**  
> МГТУ им. Н.Э. Баумана, 2026

## Авторы

| Студент | Группа |
|---------|--------|
| Уэкслер Д.С. | СМ7и-64Б |
| Рахматулла Суруш | СМ7и-64Б |
| Ван Тинхао | СМ7и-64Б |
| Хохлов В. | СМ7-64Б |

**Научный руководитель:** Вассуф Я.

---

## Описание

Система автономной навигации мобильного робота **TurtleBot3 Waffle** в симуляторе **Gazebo Harmonic** на базе **ROS2 Jazzy**.

- **Глобальный планировщик:** RRT (Rapidly-exploring Random Tree)
- **Локальный контроллер:** Pure Pursuit  
- **Карта:** строится в реальном времени из данных лидара (Occupancy Grid 400×400, 5 см/клетку)
- **Детектор застревания:** автоматическое перепланирование при отсутствии прогресса

---

## Архитектура

```
Gazebo Harmonic
  ├── /scan   (LaserScan)    ──▶┐
  ├── /odom   (Odometry)    ──▶│  global_planner_node (C++)
  └── /cmd_vel (TwistStamped)◀─┤    ├── updateMapWithLaser()
                                │    ├── planRRT()
RViz2                           │    └── purePursuitControl()
  ├── /goal_pose ─────────────▶┘
  ├── /map        ◀────────────
  └── /global_plan ◀───────────
```

---

## Запуск через Docker (рекомендуется)

### Требования

- Ubuntu 22.04 / 24.04
- Docker Engine 24+
- docker-compose v2

### Установка Docker (если не установлен)

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
newgrp docker
```

### Шаг 1 — Клонировать репозиторий

```bash
git clone https://github.com/Wexito11/ros2_navigation_project_RRT.git
cd ros2_navigation_project_RRT
```

### Шаг 2 — Разрешить GUI (Gazebo, RViz)

```bash
xhost +local:docker
```

> Эту команду нужно выполнять **каждый раз** после перезагрузки.

### Шаг 3 — Собрать и запустить

```bash
docker-compose up --build
```

Первый запуск занимает 5–10 минут (скачивает образ и собирает пакет).  
При повторных запусках — быстрее:

```bash
docker-compose up
```

### Шаг 4 — Открыть RViz (в отдельном терминале)

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2
```

В RViz:
1. `Fixed Frame` → `map`
2. `Add` → `Map` → топик `/map`
3. `Add` → `Path` → топик `/global_plan`
4. Нажать **2D Goal Pose** и кликнуть на свободной области карты

### Остановка

```bash
docker-compose down
```

---

## Запуск без Docker

pkill -f gz; pkill -f gazebo; pkill -f global_planner; pkill -f rviz2
sleep 3

### Требования

```bash
sudo apt install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim
```

### Сборка

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Wexito11/ros2_navigation_project_RRT.git ros2_navigation_project
cd ~/ros2_ws
colcon build --packages-select ros2_navigation_project
source install/setup.bash
```

### Запуск

**Терминал 1 — Симулятор:**
```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Терминал 2 — Нода (подождать 15 сек):**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_navigation_project global_planner_node
```

**Терминал 3 — RViz:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2
```

---

## Топики

| Топик | Тип | Описание |
|-------|-----|----------|
| `/scan` | `sensor_msgs/LaserScan` | Данные лидара (360 лучей) |
| `/odom` | `nav_msgs/Odometry` | Одометрия робота |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Целевая точка из RViz |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Команды скорости (Jazzy) |
| `/map` | `nav_msgs/OccupancyGrid` | Карта препятствий |
| `/global_plan` | `nav_msgs/Path` | Путь RRT |

---

## Параметры алгоритма RRT

| Параметр | Значение | Описание |
|----------|----------|----------|
| `MAX_ITER` | 10000 | Максимум итераций |
| `STEP_SIZE` | 0.30 м | Длина одного шага |
| `GOAL_BIAS` | 0.15 | Вероятность броска к цели |
| `GOAL_THRESHOLD` | 0.35 м | Радиус достижения цели |
| Inflate | 3 клетки | Раздувание препятствий (~15 см) |

---

## Диагностика

```bash
# Проверить тип cmd_vel
ros2 topic info /cmd_vel -v

# Проверить одометрию
ros2 topic echo /odom --once

# Проверить TF дерево
ros2 run tf2_tools view_frames

# Проверить список топиков
ros2 topic list
```
