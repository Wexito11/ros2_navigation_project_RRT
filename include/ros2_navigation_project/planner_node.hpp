#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <deque>
#include <mutex>

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode();
  ~GlobalPlannerNode();

private:
  // ROS коммуникации
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;

  // Данные робота
  double current_x_, current_y_, current_yaw_;
  std::vector<float> scan_ranges_;
  double goal_x_, goal_y_;
  bool goal_received_;
  bool odom_received_ = false;
  std::mutex map_mutex_;
  

  // Карта (динамически обновляемая из лидара)
  nav_msgs::msg::OccupancyGrid global_map_;
  int map_width_, map_height_;
  double map_resolution_;
  double map_origin_x_, map_origin_y_;
  bool map_initialized_;
  bool map_initial_cleared_;
  // План
  std::deque<std::pair<double, double>> global_plan_;

  // Callbacks
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void controlLoop();
  void publishMap();

  // Работа с картой
  void initMap();
  void worldToMap(double wx, double wy, int &mx, int &my) const;
  void mapToWorld(int mx, int my, double &wx, double &wy) const;
  bool isFree(int mx, int my) const;
  void updateMapWithLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  // Глобальный A*
  bool planAStar(double start_x, double start_y, double goal_x, double goal_y,
                 std::deque<std::pair<double,double>> &path);

  bool planRRT(double start_x, double start_y, double goal_x, double goal_y,
                std::deque<std::pair<double,double>> &path);

  // Локальный планировщик (Pure Pursuit)
  void purePursuitControl(double &linear, double &angular);
  void publishVelocity(double linear, double angular);
  void publishPath();
};

#endif
