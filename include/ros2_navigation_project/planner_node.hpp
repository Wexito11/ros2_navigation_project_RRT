#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <vector>
#include <deque>
#include <mutex>
#include <memory>

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode();
  ~GlobalPlannerNode() = default;

private:
  // ── Subscriptions ──────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // ── Publishers ─────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               path_pub_;

  // ── Timers ─────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr control_timer_;   // 100 ms
  rclcpp::TimerBase::SharedPtr map_timer_;       // 1 s

  // ── TF ─────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

  // ── Robot state ────────────────────────────────────────────────
  double current_x_   = 0.0;
  double current_y_   = 0.0;
  double current_yaw_ = 0.0;
  bool   odom_received_       = false;
  bool   map_initial_cleared_ = false;

  std::vector<float> scan_ranges_;

  // ── Goal ───────────────────────────────────────────────────────
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  bool   goal_received_ = false;

  // ── Occupancy map ──────────────────────────────────────────────
  mutable std::mutex              map_mutex_;
  nav_msgs::msg::OccupancyGrid    global_map_;
  int    map_width_      = 0;
  int    map_height_     = 0;
  double map_resolution_ = 0.05;
  double map_origin_x_   = -10.0;
  double map_origin_y_   = -10.0;
  bool   map_initialized_ = false;

  // ── Global path ────────────────────────────────────────────────
  std::deque<std::pair<double,double>> global_plan_;

  // ── Stuck detection ────────────────────────────────────────────
  double last_dist_to_goal_ = 1e9;
  int    stuck_counter_     = 0;

  // ── Callbacks ──────────────────────────────────────────────────
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // ── Map helpers ────────────────────────────────────────────────
  void initMap();
  void updateMapWithLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void worldToMap(double wx, double wy, int &mx, int &my) const;
  void mapToWorld(int mx, int my, double &wx, double &wy) const;
  bool isFree(int mx, int my) const;
  void clearAroundPoint(double wx, double wy, double radius_m);

  // ── Planners ───────────────────────────────────────────────────
  bool planRRT(double sx, double sy, double gx, double gy,
               std::deque<std::pair<double,double>> &path);

  // kept for completeness; not used in navigation loop
  bool planAStar(double sx, double sy, double gx, double gy,
                 std::deque<std::pair<double,double>> &path);

  // ── Control ────────────────────────────────────────────────────
  void controlLoop();
  void purePursuitControl(double &linear, double &angular);
  void publishVelocity(double linear, double angular);

  // ── Publish helpers ────────────────────────────────────────────
  void publishMap();
  void publishPath();
};

#endif  // PLANNER_NODE_HPP
