#include "ros2_navigation_project/planner_node.hpp"
#include <cmath>
#include <algorithm>
#include <queue>
#include <vector>
#include <chrono>
#include <mutex>
#include <random>
#include <tf2_ros/transform_broadcaster.h>

struct AStarNode {
  int x, y; double g, h; AStarNode* parent;
  AStarNode(int _x, int _y) : x(_x), y(_y), g(0), h(0), parent(nullptr) {}
  double f() const { return g + h; }
};
struct CompareAStar {
  bool operator()(const AStarNode* a, const AStarNode* b) { return a->f() > b->f(); }
};
struct RRTNode {
  double x, y; int parent;
  RRTNode(double _x, double _y, int _p=-1) : x(_x), y(_y), parent(_p) {}
};

GlobalPlannerNode::GlobalPlannerNode()
: Node("global_planner_node"), goal_received_(false), map_initialized_(false),
  map_initial_cleared_(false), odom_received_(false), last_x_(0), last_y_(0), stuck_counter_(0)
{
  current_x_ = current_y_ = current_yaw_ = 0.0;
  goal_x_ = goal_y_ = 0.0;

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&GlobalPlannerNode::odomCallback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&GlobalPlannerNode::scanCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10, std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  map_pub_     = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
  path_pub_    = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 1);

  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&GlobalPlannerNode::controlLoop, this));
  map_timer_ = this->create_wall_timer(std::chrono::seconds(1),
    std::bind(&GlobalPlannerNode::publishMap, this));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  initMap();
  RCLCPP_INFO(this->get_logger(), "Node started. Set goal in RViz (2D Goal Pose).");
}
GlobalPlannerNode::~GlobalPlannerNode() {}

void GlobalPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_received_ = true;
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  double siny = 2.0*(msg->pose.pose.orientation.w*msg->pose.pose.orientation.z+
                     msg->pose.pose.orientation.x*msg->pose.pose.orientation.y);
  double cosy = 1.0-2.0*(msg->pose.pose.orientation.y*msg->pose.pose.orientation.y+
                          msg->pose.pose.orientation.z*msg->pose.pose.orientation.z);
  current_yaw_ = std::atan2(siny, cosy);

  if (!map_initial_cleared_ && map_initialized_) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    int cx, cy; worldToMap(current_x_, current_y_, cx, cy);
    int r = static_cast<int>(1.5/map_resolution_);
    for (int i=-r;i<=r;++i) for (int j=-r;j<=r;++j) {
      int nx=cx+i, ny=cy+j;
      if (nx>=0&&nx<map_width_&&ny>=0&&ny<map_height_&&global_map_.data[ny*map_width_+nx]!=100)
        global_map_.data[ny*map_width_+nx]=0;
    }
    map_initial_cleared_=true;
    RCLCPP_INFO(get_logger(),"Initial free space around robot at (%.2f,%.2f)",current_x_,current_y_);
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
  global_plan_.clear();
  stuck_counter_ = 0;

  // Resetear mapa al recibir nueva meta
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    std::fill(global_map_.data.begin(), global_map_.data.end(), -1);
    int cx, cy; worldToMap(current_x_, current_y_, cx, cy);
    int r = static_cast<int>(1.5/map_resolution_);
    for (int i=-r;i<=r;++i) for (int j=-r;j<=r;++j) {
      int nx=cx+i, ny=cy+j;
      if (nx>=0&&nx<map_width_&&ny>=0&&ny<map_height_)
        global_map_.data[ny*map_width_+nx]=0;
    }
  }

  RCLCPP_INFO(this->get_logger(),"New goal: (%.2f, %.2f). Map reset. Planning RRT...", goal_x_, goal_y_);
  if (map_initialized_) {
    if (planRRT(current_x_, current_y_, goal_x_, goal_y_, global_plan_)) {
      goal_received_ = true;
      RCLCPP_INFO(this->get_logger(),"RRT path found (%zu points)", global_plan_.size());
      publishPath();
    } else {
      goal_received_ = false;
      RCLCPP_WARN(this->get_logger(),"No path found! Try another spot.");
    }
  }
}

void GlobalPlannerNode::initMap()
{
  map_resolution_=0.05; map_width_=400; map_height_=400;
  map_origin_x_=-10.0; map_origin_y_=-10.0;
  global_map_.header.frame_id="map";
  global_map_.info.resolution=map_resolution_;
  global_map_.info.width=map_width_; global_map_.info.height=map_height_;
  global_map_.info.origin.position.x=map_origin_x_;
  global_map_.info.origin.position.y=map_origin_y_;
  global_map_.info.origin.orientation.w=1.0;
  global_map_.data.resize(map_width_*map_height_,-1);
  map_initialized_=true;
}

void GlobalPlannerNode::worldToMap(double wx,double wy,int &mx,int &my) const {
  mx=static_cast<int>((wx-map_origin_x_)/map_resolution_);
  my=static_cast<int>((wy-map_origin_y_)/map_resolution_);
  mx=std::max(0,std::min(mx,map_width_-1));
  my=std::max(0,std::min(my,map_height_-1));
}
void GlobalPlannerNode::mapToWorld(int mx,int my,double &wx,double &wy) const {
  wx=map_origin_x_+(mx+0.5)*map_resolution_;
  wy=map_origin_y_+(my+0.5)*map_resolution_;
}
bool GlobalPlannerNode::isFree(int mx,int my) const {
  if (mx<0||mx>=map_width_||my<0||my>=map_height_) return false;
  int v=global_map_.data[my*map_width_+mx];
  return (v!=100&&v!=99);
}

void GlobalPlannerNode::updateMapWithLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if (!map_initialized_) return;
  std::lock_guard<std::mutex> lock(map_mutex_);
  double angle_min=scan->angle_min, angle_inc=scan->angle_increment;
  float range_max=scan->range_max;
  for (size_t i=0;i<scan->ranges.size();i+=3) {
    float r=scan->ranges[i];
    if (r<scan->range_min||r>range_max) continue;
    double angle=angle_min+i*angle_inc;
    double wx=current_x_+r*std::cos(current_yaw_+angle);
    double wy=current_y_+r*std::sin(current_yaw_+angle);
    int mx,my; worldToMap(wx,wy,mx,my);
    if (mx>=0&&mx<map_width_&&my>=0&&my<map_height_) {
      global_map_.data[my*map_width_+mx]=100;
      int inf=4;
      for(int ii=-inf;ii<=inf;++ii) for(int jj=-inf;jj<=inf;++jj) {
        int imx=mx+ii, imy=my+jj;
        if(imx>=0&&imx<map_width_&&imy>=0&&imy<map_height_&&global_map_.data[imy*map_width_+imx]!=100)
          global_map_.data[imy*map_width_+imx]=99;
      }
    }
    double ox=current_x_,oy=current_y_,dx=wx-ox,dy=wy-oy;
    double dist=std::hypot(dx,dy);
    int steps=std::max(1,(int)(dist/map_resolution_));
    for (int s=1;s<steps;++s) {
      double fx=ox+dx*s/steps, fy=oy+dy*s/steps;
      int fxi,fyi; worldToMap(fx,fy,fxi,fyi);
      if (fxi>=0&&fxi<map_width_&&fyi>=0&&fyi<map_height_&&global_map_.data[fyi*map_width_+fxi]!=100)
        global_map_.data[fyi*map_width_+fxi]=0;
    }
  }
}

bool GlobalPlannerNode::planRRT(double start_x,double start_y,double goal_x,double goal_y,
                                std::deque<std::pair<double,double>> &path)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  const int MAX_ITER=9000;
  const double STEP_SIZE=0.3, GOAL_THRESHOLD=0.35, GOAL_BIAS=0.15;
  const double map_w=map_width_*map_resolution_, map_h=map_height_*map_resolution_;
  std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> rdx(map_origin_x_,map_origin_x_+map_w);
  std::uniform_real_distribution<double> rdy(map_origin_y_,map_origin_y_+map_h);
  std::uniform_real_distribution<double> rdb(0.0,1.0);

  int smx,smy; worldToMap(start_x,start_y,smx,smy);
  int r=static_cast<int>(0.5/map_resolution_);
  for (int i=-r;i<=r;++i) for (int j=-r;j<=r;++j) {
    int cx=smx+i,cy=smy+j;
    if (cx>=0&&cx<map_width_&&cy>=0&&cy<map_height_)
      global_map_.data[cy*map_width_+cx]=0;
  }

  int gmx,gmy; worldToMap(goal_x,goal_y,gmx,gmy);
  r=static_cast<int>(0.4/map_resolution_);
  for (int i=-r;i<=r;++i) for (int j=-r;j<=r;++j) {
    int cx=gmx+i,cy=gmy+j;
    if (cx>=0&&cx<map_width_&&cy>=0&&cy<map_height_&&global_map_.data[cy*map_width_+cx]!=100)
      global_map_.data[cy*map_width_+cx]=0;
  }

  std::vector<RRTNode> tree; tree.reserve(MAX_ITER);
  tree.emplace_back(start_x,start_y,-1);

  for (int iter=0;iter<MAX_ITER;++iter) {
    double rx,ry;
    if (rdb(rng)<GOAL_BIAS){rx=goal_x;ry=goal_y;}
    else{rx=rdx(rng);ry=rdy(rng);}

    int near=0; double md=std::hypot(tree[0].x-rx,tree[0].y-ry);
    for (int i=1;i<(int)tree.size();++i){
      double d=std::hypot(tree[i].x-rx,tree[i].y-ry);
      if(d<md){md=d;near=i;}
    }

    double ddx=rx-tree[near].x, ddy=ry-tree[near].y;
    double dist=std::hypot(ddx,ddy);
    if(dist<1e-6) continue;
    double nx=tree[near].x+(ddx/dist)*STEP_SIZE;
    double ny=tree[near].y+(ddy/dist)*STEP_SIZE;

    int nmx,nmy; worldToMap(nx,ny,nmx,nmy);
    if(!isFree(nmx,nmy)) continue;

    bool col=false;
    int steps=std::max(2,(int)(STEP_SIZE/map_resolution_));
    for(int s=1;s<=steps;++s){
      double fx=tree[near].x+(ddx/dist)*STEP_SIZE*s/steps;
      double fy=tree[near].y+(ddy/dist)*STEP_SIZE*s/steps;
      int fmx,fmy; worldToMap(fx,fy,fmx,fmy);
      if(!isFree(fmx,fmy)){col=true;break;}
    }
    if(col) continue;

    tree.emplace_back(nx,ny,near);
    int new_idx=tree.size()-1;

    if(std::hypot(nx-goal_x,ny-goal_y)<GOAL_THRESHOLD){
      int cur=new_idx;
      while(cur!=-1){path.push_front({tree[cur].x,tree[cur].y});cur=tree[cur].parent;}
      path.push_back({goal_x,goal_y});
      RCLCPP_INFO(get_logger(),"RRT: path in %d iters, %zu nodes",iter,tree.size());
      return true;
    }
  }
  RCLCPP_WARN(get_logger(),"RRT: no path after %d iters",MAX_ITER);
  return false;
}

bool GlobalPlannerNode::planAStar(double sx,double sy,double gx,double gy,
                                  std::deque<std::pair<double,double>> &path)
{
  (void)sx;(void)sy;(void)gx;(void)gy;(void)path;
  return false;
}

void GlobalPlannerNode::publishPath()
{
  if(!path_pub_||global_plan_.empty()) return;
  nav_msgs::msg::Path pm; pm.header.frame_id="map"; pm.header.stamp=this->now();
  for(const auto&p:global_plan_){
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x=p.first; ps.pose.position.y=p.second;
    ps.pose.orientation.w=1.0; pm.poses.push_back(ps);
  }
  path_pub_->publish(pm);
}

void GlobalPlannerNode::purePursuitControl(double &linear,double &angular)
{
  if(global_plan_.empty()){linear=0.0;angular=0.0;return;}
  size_t nearest=0; double min_dist=1e9;
  for(size_t i=0;i<global_plan_.size();++i){
    double d=(global_plan_[i].first-current_x_)*(global_plan_[i].first-current_x_)+
             (global_plan_[i].second-current_y_)*(global_plan_[i].second-current_y_);
    if(d<min_dist){min_dist=d;nearest=i;}
  }
  double ld=0.5; size_t ti=nearest; double accum=0.0;
  for(size_t i=nearest;i<global_plan_.size()-1;++i){
    double ddx=global_plan_[i+1].first-global_plan_[i].first;
    double ddy=global_plan_[i+1].second-global_plan_[i].second;
    double sl=std::hypot(ddx,ddy);
    if(accum+sl>=ld){
      double ratio=(ld-accum)/sl;
      global_plan_[i]={global_plan_[i].first+ratio*ddx,global_plan_[i].second+ratio*ddy};
      ti=i; break;
    }
    accum+=sl; ti=i+1;
  }
  double tx=global_plan_[ti].first, ty=global_plan_[ti].second;
  double ddx=tx-current_x_, ddy=ty-current_y_;
  double dist=std::hypot(ddx,ddy);
  double at=std::atan2(ddy,ddx);
  double ad=at-current_yaw_;
  while(ad>M_PI)ad-=2*M_PI; while(ad<-M_PI)ad+=2*M_PI;
  linear=std::max(0.05,std::min(0.25,dist*0.8));
  angular=std::max(-0.8,std::min(0.8,ad*1.5));
  if(!scan_ranges_.empty()){
    int c=scan_ranges_.size()/2;
    float fr=scan_ranges_[c];
    if(fr<0.4&&fr>0.05){
      linear=0.0;
      float l=scan_ranges_[std::max(0,c-30)];
      float rv=scan_ranges_[std::min((int)scan_ranges_.size()-1,c+30)];
      angular=(l>rv)?-0.6:0.6;
    } else if(fr<0.6) linear*=(fr/0.6);
  }
}

void GlobalPlannerNode::publishVelocity(double linear,double angular)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp=this->now(); cmd.header.frame_id="base_link";
  cmd.twist.linear.x=linear; cmd.twist.angular.z=angular;
  cmd_vel_pub_->publish(cmd);
}

void GlobalPlannerNode::controlLoop()
{
  if(!odom_received_){
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Waiting for odometry...");
    return;
  }
  if(!goal_received_){publishVelocity(0.0,0.0);return;}

  double dist=std::hypot(goal_x_-current_x_,goal_y_-current_y_);
  if(dist<0.2){
    publishVelocity(0.0,0.0);
    RCLCPP_INFO(get_logger(),"Goal reached!");
    goal_received_=false; global_plan_.clear(); stuck_counter_=0;
    nav_msgs::msg::Path ep; ep.header.frame_id="map"; ep.header.stamp=now();
    path_pub_->publish(ep);
    return;
  }

  static double last_dist_to_goal = 0.0;
  double dist_diff = std::abs(last_dist_to_goal - dist);
  last_dist_to_goal = dist;
  if(dist_diff < 0.005) stuck_counter_++;
  else stuck_counter_ = 0;

  if(stuck_counter_ > 30) {
    RCLCPP_WARN(get_logger(),"Robot stuck! Replanning RRT...");
    global_plan_.clear();
    stuck_counter_ = 0;
    // Limpiar mapa alrededor del robot
    std::lock_guard<std::mutex> lock(map_mutex_);
    int cx,cy; worldToMap(current_x_,current_y_,cx,cy);
    int r=static_cast<int>(0.8/map_resolution_);
    for(int i=-r;i<=r;++i) for(int j=-r;j<=r;++j){
      int nx=cx+i,ny=cy+j;
      if(nx>=0&&nx<map_width_&&ny>=0&&ny<map_height_)
        global_map_.data[ny*map_width_+nx]=0;
    }
    return;
  }

  if(global_plan_.empty()&&map_initialized_){
    if(planRRT(current_x_,current_y_,goal_x_,goal_y_,global_plan_)){
      publishPath();
    } else {
      RCLCPP_WARN(get_logger(),"Replanning failed! Set new goal.");
      goal_received_=false;
      publishVelocity(0.0,0.0);
      return;
    }
  }
  double lin,ang;
  purePursuitControl(lin,ang);
  publishVelocity(lin,ang);
}

void GlobalPlannerNode::publishMap()
{
  if(!map_initialized_) return;
  std::lock_guard<std::mutex> lock(map_mutex_);
  global_map_.header.stamp=now();
  map_pub_->publish(global_map_);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(t);
}

int main(int argc,char**argv)
{
  rclcpp::init(argc,argv);
  auto node=std::make_shared<GlobalPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
