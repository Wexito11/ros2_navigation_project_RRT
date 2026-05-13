#include "ros2_navigation_project/planner_node.hpp"
#include <cmath>
#include <algorithm>
#include <queue>
#include <vector>
#include <chrono>
#include <mutex>

// ========================== A* структуры ==========================
struct AStarNode {
  int x, y;
  double g, h;
  AStarNode* parent;
  AStarNode(int _x, int _y) : x(_x), y(_y), g(0), h(0), parent(nullptr) {}
  double f() const { return g + h; }
};

struct CompareAStar {
  bool operator()(const AStarNode* a, const AStarNode* b) { return a->f() > b->f(); }
};

// ========================== Конструктор / деструктор ==========================
GlobalPlannerNode::GlobalPlannerNode()
: Node("global_planner_node"), goal_received_(false), map_initialized_(false), map_initial_cleared_(false)
{
  current_x_ = current_y_ = current_yaw_ = 0.0;
  goal_x_ = goal_y_ = 0.0;

  // Подписки
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&GlobalPlannerNode::odomCallback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&GlobalPlannerNode::scanCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10, std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));

  // Издатели
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 1);

  // Таймеры
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&GlobalPlannerNode::controlLoop, this));
  map_timer_ = this->create_wall_timer(std::chrono::seconds(1),
    std::bind(&GlobalPlannerNode::publishMap, this));

  // Статический tf map -> odom
  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(t);

  initMap();
  RCLCPP_INFO(this->get_logger(), "Node started. Set goal in RViz (2D Goal Pose).");
}

GlobalPlannerNode::~GlobalPlannerNode() {}

// ========================== Callbacks ==========================
void GlobalPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                            msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                  msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
  current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

  // Очищаем область вокруг робота при первой одометрии
  if (!map_initial_cleared_ && map_initialized_) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    int cx, cy;
    worldToMap(current_x_, current_y_, cx, cy);
    int radius = static_cast<int>(1.5 / map_resolution_);
    for (int i = -radius; i <= radius; ++i) {
      for (int j = -radius; j <= radius; ++j) {
        int nx = cx + i;
        int ny = cy + j;
        if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
          if (global_map_.data[ny * map_width_ + nx] != 100)
            global_map_.data[ny * map_width_ + nx] = 0;
        }
      }
    }
    map_initial_cleared_ = true;
    RCLCPP_INFO(get_logger(), "Initial free space added around robot at (%.2f, %.2f)", current_x_, current_y_);
  }
}

void GlobalPlannerNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_ranges_ = msg->ranges;
  updateMapWithLaser(msg);
}

void GlobalPlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_x_ = msg->pose.position.x;
  goal_y_ = msg->pose.position.y;
  goal_received_ = true;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f). Planning A*...", goal_x_, goal_y_);
  global_plan_.clear();
  if (map_initialized_) {
    if (planAStar(current_x_, current_y_, goal_x_, goal_y_, global_plan_)) {
      RCLCPP_INFO(this->get_logger(), "A* path found (%zu points)", global_plan_.size());
      publishPath();
    } else {
      RCLCPP_WARN(this->get_logger(), "No path to goal!");
    }
  }
}

// ========================== Инициализация карты ==========================
void GlobalPlannerNode::initMap()
{
  map_resolution_ = 0.05;
  map_width_ = 600;
  map_height_ = 600;
  map_origin_x_ = -15.0;
  map_origin_y_ = -15.0;

  global_map_.header.frame_id = "map";
  global_map_.info.resolution = map_resolution_;
  global_map_.info.width = map_width_;
  global_map_.info.height = map_height_;
  global_map_.info.origin.position.x = map_origin_x_;
  global_map_.info.origin.position.y = map_origin_y_;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.resize(map_width_ * map_height_, -1);
  map_initialized_ = true;
}

void GlobalPlannerNode::worldToMap(double wx, double wy, int &mx, int &my) const
{
  mx = static_cast<int>((wx - map_origin_x_) / map_resolution_);
  my = static_cast<int>((wy - map_origin_y_) / map_resolution_);
  mx = std::max(0, std::min(mx, map_width_-1));
  my = std::max(0, std::min(my, map_height_-1));
}

void GlobalPlannerNode::mapToWorld(int mx, int my, double &wx, double &wy) const
{
  wx = map_origin_x_ + (mx + 0.5) * map_resolution_;
  wy = map_origin_y_ + (my + 0.5) * map_resolution_;
}

bool GlobalPlannerNode::isFree(int mx, int my) const
{
  if (mx<0 || mx>=map_width_ || my<0 || my>=map_height_) return false;
  int val = global_map_.data[my * map_width_ + mx];
  // 0 = свободно, -1 = неизвестно (считаем проходимым для планирования)
  return (val == 0 || val == -1);
}

// ===== Динамическое построение карты из лидара =====
void GlobalPlannerNode::updateMapWithLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if (!map_initialized_) return;
  std::lock_guard<std::mutex> lock(map_mutex_);
  
  double angle_min = scan->angle_min;
  double angle_inc = scan->angle_increment;
  float range_max = scan->range_max;
  
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float r = scan->ranges[i];
    if (r < scan->range_min || r > range_max) continue;
    
    double angle = angle_min + i * angle_inc;
    double wx = current_x_ + r * std::cos(current_yaw_ + angle);
    double wy = current_y_ + r * std::sin(current_yaw_ + angle);
    int mx, my;
    worldToMap(wx, wy, mx, my);
    if (mx>=0 && mx<map_width_ && my>=0 && my<map_height_) {
      global_map_.data[my * map_width_ + mx] = 100; // занято
    }
    
    // Закрашиваем свободное пространство вдоль луча
    double ox = current_x_, oy = current_y_;
    double dx = wx - ox, dy = wy - oy;
    double dist = std::hypot(dx, dy);
    int steps = std::max(1, (int)(dist / map_resolution_));
    for (int s = 1; s < steps; ++s) {
      double fx = ox + dx * s / steps;
      double fy = oy + dy * s / steps;
      int fxi, fyi;
      worldToMap(fx, fy, fxi, fyi);
      if (fxi>=0 && fxi<map_width_ && fyi>=0 && fyi<map_height_) {
        if (global_map_.data[fyi * map_width_ + fxi] != 100)
          global_map_.data[fyi * map_width_ + fxi] = 0;
      }
    }
  }
}

// ========================== A* глобальный ==========================
bool GlobalPlannerNode::planAStar(double start_x, double start_y, double goal_x, double goal_y,
                                  std::deque<std::pair<double,double>> &path)
{
  int smx, smy, gmx, gmy;
  worldToMap(start_x, start_y, smx, smy);
  worldToMap(goal_x, goal_y, gmx, gmy);
  
  int start_val = global_map_.data[smy * map_width_ + smx];
  int goal_val = global_map_.data[gmy * map_width_ + gmx];
  RCLCPP_INFO(get_logger(), "Start (%d,%d)=%d, Goal (%d,%d)=%d", smx, smy, start_val, gmx, gmy, goal_val);
  
  if (!isFree(gmx, gmy)) {
    RCLCPP_WARN(get_logger(), "Goal cell not free (value=%d)", goal_val);
    return false;
  }
  if (!isFree(smx, smy)) {
    RCLCPP_WARN(get_logger(), "Start cell not free (value=%d), forcing to 0", start_val);
    global_map_.data[smy * map_width_ + smx] = 0;
  }

  std::priority_queue<AStarNode*, std::vector<AStarNode*>, CompareAStar> open;
  std::vector<std::vector<bool>> closed(map_height_, std::vector<bool>(map_width_, false));
  std::vector<std::vector<double>> g_score(map_height_, std::vector<double>(map_width_, 1e9));
  std::vector<std::vector<AStarNode*>> nodes(map_height_, std::vector<AStarNode*>(map_width_, nullptr));

  auto start_node = new AStarNode(smx, smy);
  start_node->g = 0;
  start_node->h = std::hypot(smx - gmx, smy - gmy);
  g_score[smy][smx] = 0;
  nodes[smy][smx] = start_node;
  open.push(start_node);

  int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
  double dir_cost[8] = {1,1,1,1,1.414,1.414,1.414,1.414};

  while (!open.empty()) {
    AStarNode* cur = open.top(); open.pop();
    if (closed[cur->y][cur->x]) continue;
    closed[cur->y][cur->x] = true;
    if (cur->x == gmx && cur->y == gmy) {
      AStarNode* p = cur;
      while (p) {
        double wx, wy; mapToWorld(p->x, p->y, wx, wy);
        path.push_front({wx, wy});
        p = p->parent;
      }
      for (auto &row : nodes) for (auto &n : row) delete n;
      return true;
    }
    for (int d=0; d<8; ++d) {
      int nx = cur->x + dirs[d][0];
      int ny = cur->y + dirs[d][1];
      if (nx<0 || nx>=map_width_ || ny<0 || ny>=map_height_) continue;
      if (!isFree(nx, ny)) continue;
      double tentative = g_score[cur->y][cur->x] + dir_cost[d];
      if (tentative < g_score[ny][nx]) {
        g_score[ny][nx] = tentative;
        if (!nodes[ny][nx]) nodes[ny][nx] = new AStarNode(nx, ny);
        nodes[ny][nx]->g = tentative;
        nodes[ny][nx]->h = std::hypot(nx - gmx, ny - gmy);
        nodes[ny][nx]->parent = cur;
        open.push(nodes[ny][nx]);
      }
    }
  }
  for (auto &row : nodes) for (auto &n : row) delete n;
  return false;
}

// ========================== Публикация пути ==========================
void GlobalPlannerNode::publishPath()
{
  if (!path_pub_ || global_plan_.empty()) return;
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = this->now();
  for (const auto& p : global_plan_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = p.first;
    pose.pose.position.y = p.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }
  path_pub_->publish(path_msg);
}

// ========================== Локальный Pure Pursuit ==========================
void GlobalPlannerNode::purePursuitControl(double &linear, double &angular)
{
  if (global_plan_.empty()) {
    linear = 0.0; angular = 0.0;
    return;
  }

  size_t nearest = 0;
  double min_dist = 1e9;
  for (size_t i = 0; i < global_plan_.size(); ++i) {
    double dx = global_plan_[i].first - current_x_;
    double dy = global_plan_[i].second - current_y_;
    double d2 = dx*dx + dy*dy;
    if (d2 < min_dist) {
      min_dist = d2;
      nearest = i;
    }
  }

  double lookahead_dist = 0.6;
  size_t target_idx = nearest;
  double accum_dist = 0.0;
  for (size_t i = nearest; i < global_plan_.size() - 1; ++i) {
    double dx = global_plan_[i+1].first - global_plan_[i].first;
    double dy = global_plan_[i+1].second - global_plan_[i].second;
    double seg_len = std::hypot(dx, dy);
    if (accum_dist + seg_len >= lookahead_dist) {
      double ratio = (lookahead_dist - accum_dist) / seg_len;
      double tx = global_plan_[i].first + ratio * dx;
      double ty = global_plan_[i].second + ratio * dy;
      target_idx = i;
      global_plan_[target_idx] = {tx, ty};
      break;
    }
    accum_dist += seg_len;
    target_idx = i+1;
  }

  double tx = global_plan_[target_idx].first;
  double ty = global_plan_[target_idx].second;

  double dx = tx - current_x_;
  double dy = ty - current_y_;
  double distance = std::hypot(dx, dy);
  double angle_to_target = std::atan2(dy, dx);
  double angle_diff = angle_to_target - current_yaw_;
  while (angle_diff > M_PI) angle_diff -= 2.0*M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0*M_PI;

  linear = std::min(0.3, distance * 0.8);
  angular = angle_diff * 1.5;
  linear = std::max(0.05, std::min(0.3, linear));
  angular = std::max(-0.8, std::min(0.8, angular));

  if (!scan_ranges_.empty()) {
    int center = scan_ranges_.size() / 2;
    float front_range = scan_ranges_[center];
    if (front_range < 0.35 && front_range > 0.05) {
      linear = 0.0;
      float left = scan_ranges_[std::max(0, center-30)];
      float right = scan_ranges_[std::min((int)scan_ranges_.size()-1, center+30)];
      angular = (left > right) ? -0.6 : 0.6;
    } else if (front_range < 0.6) {
      linear *= (front_range / 0.6);
    }
  }
}

void GlobalPlannerNode::publishVelocity(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_vel_pub_->publish(cmd);
}

void GlobalPlannerNode::controlLoop()
{
  static bool warned = false;
  if (std::abs(current_x_) < 0.01 && std::abs(current_y_) < 0.01 && !warned) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for odometry...");
    warned = true;
    return;
  }
  warned = false;

  if (!goal_received_) {
    publishVelocity(0.0, 0.0);
    return;
  }

  double dist_to_goal = std::hypot(goal_x_ - current_x_, goal_y_ - current_y_);
  if (dist_to_goal < 0.2) {
    publishVelocity(0.0, 0.0);
    RCLCPP_INFO(get_logger(), "Goal reached!");
    goal_received_ = false;
    global_plan_.clear();
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = "map";
    empty_path.header.stamp = now();
    path_pub_->publish(empty_path);
    return;
  }

  if (global_plan_.empty() && map_initialized_) {
    if (planAStar(current_x_, current_y_, goal_x_, goal_y_, global_plan_)) {
      publishPath();
    } else {
      RCLCPP_WARN(get_logger(), "Replanning failed");
    }
  }

  double linear, angular;
  purePursuitControl(linear, angular);
  publishVelocity(linear, angular);
}

void GlobalPlannerNode::publishMap()
{
  if (!map_initialized_) return;
  std::lock_guard<std::mutex> lock(map_mutex_);
  global_map_.header.stamp = now();
  map_pub_->publish(global_map_);
}

// ========================== main ==========================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
