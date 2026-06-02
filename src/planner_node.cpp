#include "ros2_navigation_project/planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <queue>
#include <random>
#include <vector>

using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════
//  RRT node structure
// ═══════════════════════════════════════════════════════════════
struct RRTNode {
  double x, y;
  int    parent;
  RRTNode(double x, double y, int parent = -1) : x(x), y(y), parent(parent) {}
};

// ═══════════════════════════════════════════════════════════════
//  A* node structure  (kept for reference, not used in main loop)
// ═══════════════════════════════════════════════════════════════
struct AStarNode {
  int x, y;
  double g, h;
  AStarNode *parent;
  AStarNode(int x, int y) : x(x), y(y), g(0), h(0), parent(nullptr) {}
  double f() const { return g + h; }
};
struct CmpAStar {
  bool operator()(const AStarNode *a, const AStarNode *b) const { return a->f() > b->f(); }
};

// ═══════════════════════════════════════════════════════════════
//  Constructor
// ═══════════════════════════════════════════════════════════════
GlobalPlannerNode::GlobalPlannerNode()
: Node("global_planner_node")
{
  // ── subscriptions ──────────────────────────────────────────
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&GlobalPlannerNode::odomCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&GlobalPlannerNode::scanCallback, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));

  // ── publishers ─────────────────────────────────────────────
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  map_pub_     = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
  path_pub_    = create_publisher<nav_msgs::msg::Path>("/global_plan", 1);

  // ── timers ─────────────────────────────────────────────────
  control_timer_ = create_wall_timer(100ms,
    std::bind(&GlobalPlannerNode::controlLoop, this));
  map_timer_ = create_wall_timer(1s,
    std::bind(&GlobalPlannerNode::publishMap, this));

  // ── static TF  map → odom ──────────────────────────────────
  tf_static_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp    = now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id  = "odom";
  tf_msg.transform.rotation.w = 1.0;
  tf_static_->sendTransform(tf_msg);

  // ── map ────────────────────────────────────────────────────
  initMap();

  RCLCPP_INFO(get_logger(), "global_planner_node ready — set goal with 2D Goal Pose in RViz");
}

// ═══════════════════════════════════════════════════════════════
//  Callbacks
// ═══════════════════════════════════════════════════════════════
void GlobalPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_received_ = true;

  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  const auto &q = msg->pose.pose.orientation;
  double siny   = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy   = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  current_yaw_  = std::atan2(siny, cosy);

  // clear initial free zone once
  if (!map_initial_cleared_ && map_initialized_) {
    clearAroundPoint(current_x_, current_y_, 1.5);
    map_initial_cleared_ = true;
    RCLCPP_INFO(get_logger(), "initial free space cleared around (%.2f, %.2f)",
                current_x_, current_y_);
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

  RCLCPP_INFO(get_logger(), "new goal (%.2f, %.2f) — resetting map and planning RRT…",
              goal_x_, goal_y_);

  // reset map so old noise doesn't block the new route
  {
    std::lock_guard<std::mutex> lk(map_mutex_);
    std::fill(global_map_.data.begin(), global_map_.data.end(), -1);
  }
  clearAroundPoint(current_x_, current_y_, 1.5);

  global_plan_.clear();
  stuck_counter_     = 0;
  last_dist_to_goal_ = 1e9;

  if (!map_initialized_) { return; }

  if (planRRT(current_x_, current_y_, goal_x_, goal_y_, global_plan_)) {
    goal_received_ = true;
    RCLCPP_INFO(get_logger(), "RRT path found (%zu points)", global_plan_.size());
    publishPath();
  } else {
    goal_received_ = false;
    RCLCPP_WARN(get_logger(), "RRT could not find a path — try a different goal");
  }
}

// ═══════════════════════════════════════════════════════════════
//  Map helpers
// ═══════════════════════════════════════════════════════════════
void GlobalPlannerNode::initMap()
{
  map_width_      = 400;
  map_height_     = 400;
  map_resolution_ = 0.05;   // 5 cm / cell
  map_origin_x_   = -10.0;
  map_origin_y_   = -10.0;

  global_map_.header.frame_id           = "map";
  global_map_.info.resolution           = map_resolution_;
  global_map_.info.width                = map_width_;
  global_map_.info.height               = map_height_;
  global_map_.info.origin.position.x    = map_origin_x_;
  global_map_.info.origin.position.y    = map_origin_y_;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(map_width_ * map_height_, -1);

  map_initialized_ = true;
}

void GlobalPlannerNode::worldToMap(double wx, double wy, int &mx, int &my) const
{
  mx = static_cast<int>((wx - map_origin_x_) / map_resolution_);
  my = static_cast<int>((wy - map_origin_y_) / map_resolution_);
  mx = std::clamp(mx, 0, map_width_  - 1);
  my = std::clamp(my, 0, map_height_ - 1);
}

void GlobalPlannerNode::mapToWorld(int mx, int my, double &wx, double &wy) const
{
  wx = map_origin_x_ + (mx + 0.5) * map_resolution_;
  wy = map_origin_y_ + (my + 0.5) * map_resolution_;
}

bool GlobalPlannerNode::isFree(int mx, int my) const
{
  if (mx < 0 || mx >= map_width_ || my < 0 || my >= map_height_) return false;
  int v = global_map_.data[my * map_width_ + mx];
  return (v != 100 && v != 99);   // 100 = obstacle, 99 = inflated
}

void GlobalPlannerNode::clearAroundPoint(double wx, double wy, double radius_m)
{
  std::lock_guard<std::mutex> lk(map_mutex_);
  int cx, cy;
  worldToMap(wx, wy, cx, cy);
  int r = static_cast<int>(radius_m / map_resolution_);
  for (int i = -r; i <= r; ++i) {
    for (int j = -r; j <= r; ++j) {
      int nx = cx + i, ny = cy + j;
      if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_)
        if (global_map_.data[ny * map_width_ + nx] != 100)
          global_map_.data[ny * map_width_ + nx] = 0;
    }
  }
}

void GlobalPlannerNode::updateMapWithLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if (!map_initialized_) return;
  std::lock_guard<std::mutex> lk(map_mutex_);

  const double angle_min = scan->angle_min;
  const double angle_inc = scan->angle_increment;
  const float  rmax      = scan->range_max;

  // sample every 3rd ray to reduce CPU load
  for (size_t i = 0; i < scan->ranges.size(); i += 3) {
    float r = scan->ranges[i];
    if (r < scan->range_min || r > rmax) continue;

    double angle = angle_min + i * angle_inc;
    double wx    = current_x_ + r * std::cos(current_yaw_ + angle);
    double wy    = current_y_ + r * std::sin(current_yaw_ + angle);

    int mx, my;
    worldToMap(wx, wy, mx, my);

    // mark obstacle + inflate 3 cells (~15 cm)
    global_map_.data[my * map_width_ + mx] = 100;
    constexpr int INF = 3;
    for (int di = -INF; di <= INF; ++di) {
      for (int dj = -INF; dj <= INF; ++dj) {
        int imx = mx + di, imy = my + dj;
        if (imx >= 0 && imx < map_width_ && imy >= 0 && imy < map_height_)
          if (global_map_.data[imy * map_width_ + imx] != 100)
            global_map_.data[imy * map_width_ + imx] = 99;
      }
    }

    // ray-trace free cells
    double ox = current_x_, oy = current_y_;
    double dx = wx - ox,    dy = wy - oy;
    double d  = std::hypot(dx, dy);
    int steps = std::max(1, static_cast<int>(d / map_resolution_));
    for (int s = 1; s < steps; ++s) {
      double fx = ox + dx * s / steps;
      double fy = oy + dy * s / steps;
      int fxi, fyi;
      worldToMap(fx, fy, fxi, fyi);
      if (fxi >= 0 && fxi < map_width_ && fyi >= 0 && fyi < map_height_)
        if (global_map_.data[fyi * map_width_ + fxi] == -1)
          global_map_.data[fyi * map_width_ + fxi] = 0;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  RRT planner
// ═══════════════════════════════════════════════════════════════
bool GlobalPlannerNode::planRRT(double sx, double sy, double gx, double gy,
                                std::deque<std::pair<double,double>> &path)
{
  std::lock_guard<std::mutex> lk(map_mutex_);

  constexpr int    MAX_ITER       = 10000;
  constexpr double STEP_SIZE      = 0.30;   // m
  constexpr double GOAL_THRESHOLD = 0.35;   // m
  constexpr double GOAL_BIAS      = 0.15;   // 15 % shots toward goal

  const double map_w = map_width_  * map_resolution_;
  const double map_h = map_height_ * map_resolution_;

  std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> rx(map_origin_x_, map_origin_x_ + map_w);
  std::uniform_real_distribution<double> ry(map_origin_y_, map_origin_y_ + map_h);
  std::uniform_real_distribution<double> rb(0.0, 1.0);

  // force start cell free
  {
    int smx, smy;
    worldToMap(sx, sy, smx, smy);
    int r = static_cast<int>(0.4 / map_resolution_);
    for (int i = -r; i <= r; ++i)
      for (int j = -r; j <= r; ++j) {
        int cx = smx + i, cy = smy + j;
        if (cx >= 0 && cx < map_width_ && cy >= 0 && cy < map_height_)
          global_map_.data[cy * map_width_ + cx] = 0;
      }
  }
  // force goal cell free
  {
    int gmx, gmy;
    worldToMap(gx, gy, gmx, gmy);
    int r = static_cast<int>(0.4 / map_resolution_);
    for (int i = -r; i <= r; ++i)
      for (int j = -r; j <= r; ++j) {
        int cx = gmx + i, cy = gmy + j;
        if (cx >= 0 && cx < map_width_ && cy >= 0 && cy < map_height_)
          if (global_map_.data[cy * map_width_ + cx] != 100)
            global_map_.data[cy * map_width_ + cx] = 0;
      }
  }

  std::vector<RRTNode> tree;
  tree.reserve(MAX_ITER);
  tree.emplace_back(sx, sy);

  for (int iter = 0; iter < MAX_ITER; ++iter) {
    // sample
    double qx, qy;
    if (rb(rng) < GOAL_BIAS) { qx = gx; qy = gy; }
    else                      { qx = rx(rng); qy = ry(rng); }

    // nearest node
    int    near = 0;
    double md   = std::hypot(tree[0].x - qx, tree[0].y - qy);
    for (int i = 1; i < static_cast<int>(tree.size()); ++i) {
      double d = std::hypot(tree[i].x - qx, tree[i].y - qy);
      if (d < md) { md = d; near = i; }
    }

    // steer
    double dx   = qx - tree[near].x;
    double dy   = qy - tree[near].y;
    double dist = std::hypot(dx, dy);
    if (dist < 1e-9) continue;
    double nx = tree[near].x + (dx / dist) * STEP_SIZE;
    double ny = tree[near].y + (dy / dist) * STEP_SIZE;

    // check new cell
    int nmx, nmy;
    worldToMap(nx, ny, nmx, nmy);
    if (!isFree(nmx, nmy)) continue;

    // collision check along edge
    int   steps     = std::max(2, static_cast<int>(STEP_SIZE / map_resolution_));
    bool  collision = false;
    for (int s = 1; s <= steps && !collision; ++s) {
      double fx = tree[near].x + (dx / dist) * STEP_SIZE * s / steps;
      double fy = tree[near].y + (dy / dist) * STEP_SIZE * s / steps;
      int fmx, fmy;
      worldToMap(fx, fy, fmx, fmy);
      if (!isFree(fmx, fmy)) collision = true;
    }
    if (collision) continue;

    tree.emplace_back(nx, ny, near);
    int new_idx = static_cast<int>(tree.size()) - 1;

    // goal check
    if (std::hypot(nx - gx, ny - gy) < GOAL_THRESHOLD) {
      // reconstruct path
      int cur = new_idx;
      while (cur != -1) {
        path.push_front({tree[cur].x, tree[cur].y});
        cur = tree[cur].parent;
      }
      path.push_back({gx, gy});
      RCLCPP_INFO(get_logger(), "RRT: path in %d iters, tree size %zu",
                  iter, tree.size());
      return true;
    }
  }

  RCLCPP_WARN(get_logger(), "RRT: no path found after %d iterations", MAX_ITER);
  return false;
}

// ─── A* (not used in main loop, kept for completeness) ─────────
bool GlobalPlannerNode::planAStar(double /*sx*/, double /*sy*/,
                                  double /*gx*/, double /*gy*/,
                                  std::deque<std::pair<double,double>> & /*path*/)
{
  return false;
}

// ═══════════════════════════════════════════════════════════════
//  Pure Pursuit local controller
// ═══════════════════════════════════════════════════════════════
void GlobalPlannerNode::purePursuitControl(double &linear, double &angular)
{
  if (global_plan_.empty()) { linear = 0.0; angular = 0.0; return; }

  // find nearest point on path
  size_t nearest = 0;
  double min_d2  = 1e18;
  for (size_t i = 0; i < global_plan_.size(); ++i) {
    double dx = global_plan_[i].first  - current_x_;
    double dy = global_plan_[i].second - current_y_;
    double d2 = dx * dx + dy * dy;
    if (d2 < min_d2) { min_d2 = d2; nearest = i; }
  }

  // lookahead 0.5 m
  constexpr double LD = 0.5;
  size_t ti    = nearest;
  double accum = 0.0;
  for (size_t i = nearest; i + 1 < global_plan_.size(); ++i) {
    double dx  = global_plan_[i+1].first  - global_plan_[i].first;
    double dy  = global_plan_[i+1].second - global_plan_[i].second;
    double seg = std::hypot(dx, dy);
    if (accum + seg >= LD) {
      double ratio = (LD - accum) / seg;
      global_plan_[i] = { global_plan_[i].first  + ratio * dx,
                          global_plan_[i].second + ratio * dy };
      ti = i;
      break;
    }
    accum += seg;
    ti = i + 1;
  }

  double tx = global_plan_[ti].first;
  double ty = global_plan_[ti].second;
  double dx = tx - current_x_;
  double dy = ty - current_y_;
  double d  = std::hypot(dx, dy);

  double angle_err = std::atan2(dy, dx) - current_yaw_;
  while (angle_err >  M_PI) angle_err -= 2.0 * M_PI;
  while (angle_err < -M_PI) angle_err += 2.0 * M_PI;

  linear  = std::clamp(d * 0.8, 0.05, 0.25);
  angular = std::clamp(angle_err * 1.5, -0.8, 0.8);

  // emergency stop if obstacle too close in front
  if (!scan_ranges_.empty()) {
    int   c  = static_cast<int>(scan_ranges_.size()) / 2;
    float fr = scan_ranges_[c];
    if (fr > 0.05f && fr < 0.35f) {
      linear  = 0.0;
      float l = scan_ranges_[std::max(0, c - 30)];
      float r = scan_ranges_[std::min(static_cast<int>(scan_ranges_.size()) - 1, c + 30)];
      angular = (l > r) ? -0.6 : 0.6;
    } else if (fr < 0.6f) {
      linear *= fr / 0.6f;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  Control loop  (10 Hz)
// ═══════════════════════════════════════════════════════════════
void GlobalPlannerNode::controlLoop()
{
  if (!odom_received_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "waiting for odometry…");
    return;
  }
  if (!goal_received_) {
    publishVelocity(0.0, 0.0);
    return;
  }

  double dist = std::hypot(goal_x_ - current_x_, goal_y_ - current_y_);

  // ── goal reached ───────────────────────────────────────────
  if (dist < 0.20) {
    publishVelocity(0.0, 0.0);
    RCLCPP_INFO(get_logger(), "goal reached!");
    goal_received_ = false;
    global_plan_.clear();
    stuck_counter_ = 0;
    nav_msgs::msg::Path empty;
    empty.header.frame_id = "map";
    empty.header.stamp    = now();
    path_pub_->publish(empty);
    return;
  }

  // ── stuck detection ────────────────────────────────────────
  double dist_diff = std::abs(last_dist_to_goal_ - dist);
  last_dist_to_goal_ = dist;
  if (dist_diff < 0.005) ++stuck_counter_;
  else                    stuck_counter_ = 0;

  if (stuck_counter_ > 40) {          // ~4 s without progress
    RCLCPP_WARN(get_logger(), "stuck detected — clearing local map and replanning");
    clearAroundPoint(current_x_, current_y_, 0.8);
    global_plan_.clear();
    stuck_counter_ = 0;
    return;
  }

  // ── replan if path is empty ────────────────────────────────
  if (global_plan_.empty()) {
    if (planRRT(current_x_, current_y_, goal_x_, goal_y_, global_plan_)) {
      publishPath();
    } else {
      RCLCPP_WARN(get_logger(), "replan failed — set a new goal");
      goal_received_ = false;
      publishVelocity(0.0, 0.0);
    }
    return;
  }

  // ── follow path ────────────────────────────────────────────
  double lin, ang;
  purePursuitControl(lin, ang);
  publishVelocity(lin, ang);
}

// ═══════════════════════════════════════════════════════════════
//  Publish helpers
// ═══════════════════════════════════════════════════════════════
void GlobalPlannerNode::publishVelocity(double linear, double angular)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp    = now();
  msg.header.frame_id = "base_link";
  msg.twist.linear.x  = linear;
  msg.twist.angular.z = angular;
  cmd_vel_pub_->publish(msg);
}

void GlobalPlannerNode::publishPath()
{
  if (global_plan_.empty()) return;
  nav_msgs::msg::Path pm;
  pm.header.frame_id = "map";
  pm.header.stamp    = now();
  for (const auto &pt : global_plan_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x    = pt.first;
    ps.pose.position.y    = pt.second;
    ps.pose.orientation.w = 1.0;
    pm.poses.push_back(ps);
  }
  path_pub_->publish(pm);
}

void GlobalPlannerNode::publishMap()
{
  if (!map_initialized_) return;
  std::lock_guard<std::mutex> lk(map_mutex_);
  global_map_.header.stamp = now();
  map_pub_->publish(global_map_);
}

// ═══════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
