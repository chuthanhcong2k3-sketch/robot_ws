#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

inline double norm_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double quat_to_yaw(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

inline geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
  tf2::Quaternion tq;
  tq.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion q;
  q.x = tq.x();
  q.y = tq.y();
  q.z = tq.z();
  q.w = tq.w();
  return q;
}

struct GridMeta {
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
};

inline bool in_bounds(int x, int y, const GridMeta &m) {
  return (x >= 0 && y >= 0 && x < m.width && y < m.height);
}

inline int to_index(int x, int y, const GridMeta &m) {
  return y * m.width + x;
}

inline std::pair<int, int> to_xy(int idx, const GridMeta &m) {
  int y = idx / m.width;
  int x = idx - y * m.width;
  return {x, y};
}

inline std::pair<int, int> meters_to_cell(double mx, double my, const GridMeta &m) {
  int x = static_cast<int>(std::floor((mx - m.origin_x) / m.resolution));
  int y = static_cast<int>(std::floor((my - m.origin_y) / m.resolution));
  return {x, y};
}

inline std::pair<double, double> cell_to_meters(int x, int y, const GridMeta &m) {
  // cell center
  double mx = m.origin_x + (static_cast<double>(x) + 0.5) * m.resolution;
  double my = m.origin_y + (static_cast<double>(y) + 0.5) * m.resolution;
  return {mx, my};
}

}  // namespace

class GridNav : public rclcpp::Node {
public:
  GridNav() : Node("grid_nav") {
    // ===== Parameters =====
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
    goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    plan_topic_ = this->declare_parameter<std::string>("plan_topic", "/plan");

    occupied_thresh_ = this->declare_parameter<int>("occupied_thresh", 50);
    unknown_is_obstacle_ = this->declare_parameter<bool>("unknown_is_obstacle", true);
    inflation_radius_m_ = this->declare_parameter<double>("inflation_radius_m", 0.18);

    plan_rate_hz_ = this->declare_parameter<double>("plan_rate_hz", 2.0);
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 20.0);

    // follower
    lookahead_m_ = this->declare_parameter<double>("lookahead_m", 0.35);
    goal_tol_m_ = this->declare_parameter<double>("goal_tol_m", 0.12);
    max_lin_ = this->declare_parameter<double>("max_lin", 0.22);
    max_ang_ = this->declare_parameter<double>("max_ang", 0.8);
    k_ang_ = this->declare_parameter<double>("k_ang", 2.0);
    k_lin_ = this->declare_parameter<double>("k_lin", 0.9);
    angle_stop_rad_ = this->declare_parameter<double>("angle_stop_rad", 0.55);

    // dynamic obstacle integration
    use_scan_obstacles_ = this->declare_parameter<bool>("use_scan_obstacles", true);
    scan_obstacle_hold_sec = this->declare_parameter<double>("scan_obstacle_hold_sec", 2.0);

    // Reaction distance in front of the robot.
    // We interpret this as the distance where we START reacting (replan + slow down),
    // not an immediate hard-stop distance.
    safe_dist_front = this->declare_parameter<double>("safe_dist_front", 2.0);
    if (safe_dist_front < 2.0) {
      RCLCPP_WARN(this->get_logger(), "safe_dist_front=%.2f < 2.0, force to 2.0 for safety", safe_dist_front);
      safe_dist_front = 2.0;
    }

    // Hard-stop distance (emergency). Default uses your robot nose offset (~1.2m).
    stop_dist_front = this->declare_parameter<double>("stop_dist_front", 1.2);
    if (stop_dist_front < 0.2) stop_dist_front = 0.2;
    if (stop_dist_front > safe_dist_front) stop_dist_front = safe_dist_front;

    // Minimum linear speed when we decide to move (helps overcome deadband / stiction)
    min_lin_move = this->declare_parameter<double>("min_lin_move", 0.12);
    min_ang_move = this->declare_parameter<double>("min_ang_move", 0.12);
    front_half_angle_deg = this->declare_parameter<double>("front_half_angle_deg", 18.0);

    // TF
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    // ===== TF listener =====
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ===== Pub/Sub =====
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    plan_pub_ = this->create_publisher<nav_msgs::msg::Path>(plan_topic_, 10);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&GridNav::onMap, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10,
        std::bind(&GridNav::onGoal, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10,
        std::bind(&GridNav::onScan, this, std::placeholders::_1));

    // timers
    plan_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(0.2, plan_rate_hz_))),
        std::bind(&GridNav::planTick, this));

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, control_rate_hz_))),
        std::bind(&GridNav::controlTick, this));

    RCLCPP_INFO(this->get_logger(),
                "grid_nav started: plan_rate=%.1fHz, control_rate=%.1fHz, inflation=%.2fm",
                plan_rate_hz_, control_rate_hz_, inflation_radius_m_);
  }

private:
  // ===================== Callbacks =====================
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    have_map_ = true;

    meta_.width = static_cast<int>(map_.info.width);
    meta_.height = static_cast<int>(map_.info.height);
    meta_.resolution = map_.info.resolution;
    meta_.origin_x = map_.info.origin.position.x;
    meta_.origin_y = map_.info.origin.position.y;

    // mark need replan if map changes while navigating
    if (goal_active_) {
      need_replan_ = true;
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_active_ = true;
    need_replan_ = true;

    // reset follower state
    path_points_.clear();
    path_cursor_ = 0;

    RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f) frame=%s",
                goal_.pose.position.x, goal_.pose.position.y,
                goal_.header.frame_id.c_str());
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = *msg;
    have_scan_ = true;
    updateFrontMin(*msg);

    if (!use_scan_obstacles_ || !goal_active_) return;

    // integrate scan points into temporary obstacle layer in map frame
    Pose2D base = {};
    if (!getRobotPose(base)) return;

    const double now_sec = this->now().seconds();
    (void)now_sec;

    const int n = static_cast<int>(msg->ranges.size());
    if (n <= 0) return;

    const double a0 = msg->angle_min;
    const double da = msg->angle_increment;

    // limit points we add to obstacles (avoid filling the whole map)
    const double r_max = std::min(3.5, static_cast<double>(msg->range_max));

    const double cy = std::cos(base.yaw);
    const double sy = std::sin(base.yaw);

    for (int i = 0; i < n; i++) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r)) continue;
      if (r <= msg->range_min) continue;
      if (r >= r_max) continue;

      const double a = a0 + static_cast<double>(i) * da;
      const double bx = static_cast<double>(r) * std::cos(a);
      const double by = static_cast<double>(r) * std::sin(a);

      // base_link -> map
      const double mx = base.x + cy * bx - sy * by;
      const double my = base.y + sy * bx + cy * by;

      auto [cx_cell, cy_cell] = meters_to_cell(mx, my, meta_);
      if (!in_bounds(cx_cell, cy_cell, meta_)) continue;
      const int idx = to_index(cx_cell, cy_cell, meta_);
      scan_obstacles_[idx] = this->now();
    }

    // if something is too close in front, request immediate replan
    if (front_min_dist_ < safe_dist_front) {
      need_replan_ = true;
    }
  }

  // ===================== Planning =====================
  void planTick() {
    if (!goal_active_) return;
    if (!have_map_) return;

    if (!need_replan_ && have_path_) {
      // still check if path has become invalid (new obstacle on path)
      if (pathBlockedAhead()) {
        need_replan_ = true;
      }
    }

    if (!need_replan_) return;

    Pose2D start_pose{};
    if (!getRobotPose(start_pose)) return;

    geometry_msgs::msg::PoseStamped goal_in_map = goal_;
    if (normalize_frame(goal_.header.frame_id) != map_frame_) {
      // Try transform goal -> map
      try {
        auto tf = tf_buffer_->lookupTransform(map_frame_, normalize_frame(goal_.header.frame_id), tf2::TimePointZero);
        tf2::doTransform(goal_, goal_in_map, tf);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Goal transform failed: %s", ex.what());
        return;
      }
    }

    std::vector<geometry_msgs::msg::PoseStamped> new_path;
    if (!computePath(start_pose, goal_in_map, new_path)) {
      // publish empty plan so UI clears
      publishPlan({});
      have_path_ = false;
      need_replan_ = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path to goal.");
      return;
    }

    path_points_ = std::move(new_path);
    path_cursor_ = 0;
    have_path_ = !path_points_.empty();
    need_replan_ = false;

    publishPlan(path_points_);
  }

  bool computePath(const Pose2D &start_pose,
                   const geometry_msgs::msg::PoseStamped &goal_pose,
                   std::vector<geometry_msgs::msg::PoseStamped> &out_path) {
    out_path.clear();

    if (!have_map_) return false;

    // Build inflated obstacle grid with temporary scan obstacles
    std::vector<uint8_t> obstacle(meta_.width * meta_.height, 0);

    const auto &data = map_.data;
    for (int i = 0; i < static_cast<int>(data.size()); i++) {
      const int8_t v = data[i];
      bool occ = false;
      if (v < 0) occ = unknown_is_obstacle_;
      else occ = (v >= occupied_thresh_);
      obstacle[i] = occ ? 1 : 0;
    }

    // merge scan obstacles with decay
    if (use_scan_obstacles_) {
      const rclcpp::Time now = this->now();
      std::vector<int> to_erase;
      to_erase.reserve(256);
      for (const auto &kv : scan_obstacles_) {
        if ((now - kv.second).seconds() > scan_obstacle_hold_sec) {
          to_erase.push_back(kv.first);
        } else {
          if (kv.first >= 0 && kv.first < static_cast<int>(obstacle.size())) {
            obstacle[kv.first] = 1;
          }
        }
      }
      for (int idx : to_erase) scan_obstacles_.erase(idx);
    }

    // Inflate by multi-source BFS distance (4-neighbor approx)
    const int infl_cells = std::max(0, static_cast<int>(std::ceil(inflation_radius_m_ / meta_.resolution)));
    if (infl_cells > 0) {
      const int N = meta_.width * meta_.height;
      std::vector<int16_t> dist(N, std::numeric_limits<int16_t>::max());
      std::queue<int> q;
      for (int i = 0; i < N; i++) {
        if (obstacle[i]) {
          dist[i] = 0;
          q.push(i);
        }
      }

      const int dx4[4] = {1, -1, 0, 0};
      const int dy4[4] = {0, 0, 1, -1};

      while (!q.empty()) {
        const int cur = q.front();
        q.pop();
        if (dist[cur] >= infl_cells) continue;

        auto [cx, cy] = to_xy(cur, meta_);
        for (int k = 0; k < 4; k++) {
          const int nx = cx + dx4[k];
          const int ny = cy + dy4[k];
          if (!in_bounds(nx, ny, meta_)) continue;
          const int ni = to_index(nx, ny, meta_);
          if (dist[ni] <= dist[cur] + 1) continue;
          dist[ni] = static_cast<int16_t>(dist[cur] + 1);
          q.push(ni);
        }
      }

      for (int i = 0; i < N; i++) {
        if (dist[i] <= infl_cells) obstacle[i] = 1;
      }
    }

    // start/goal cells
    auto [sx, sy] = meters_to_cell(start_pose.x, start_pose.y, meta_);
    auto [gx, gy] = meters_to_cell(goal_pose.pose.position.x, goal_pose.pose.position.y, meta_);

    if (!in_bounds(sx, sy, meta_) || !in_bounds(gx, gy, meta_)) {
      RCLCPP_WARN(this->get_logger(), "Start/goal out of map bounds");
      return false;
    }

    int sidx = to_index(sx, sy, meta_);
    int gidx = to_index(gx, gy, meta_);

    // If goal is in obstacle, snap to nearest free cell
    if (obstacle[gidx]) {
      gidx = nearestFree(gidx, obstacle);
      if (gidx < 0) return false;
      auto [ngx, ngy] = to_xy(gidx, meta_);
      auto [gmx, gmy] = cell_to_meters(ngx, ngy, meta_);
      // update goal_pose copy so follower aims at reachable point
      (void)gmx;
      (void)gmy;
    }

    if (obstacle[sidx]) {
      sidx = nearestFree(sidx, obstacle);
      if (sidx < 0) return false;
    }

    // A* search
    const int N = meta_.width * meta_.height;
    std::vector<float> gscore(N, std::numeric_limits<float>::infinity());
    std::vector<int> came_from(N, -1);

    auto h = [&](int idx) -> float {
      auto [x, y] = to_xy(idx, meta_);
      auto [tx, ty] = to_xy(gidx, meta_);
      const float dx = static_cast<float>(x - tx);
      const float dy = static_cast<float>(y - ty);
      return std::sqrt(dx * dx + dy * dy);
    };

    struct QN {
      float f;
      int idx;
    };
    struct QCmp {
      bool operator()(const QN &a, const QN &b) const { return a.f > b.f; }
    };

    std::priority_queue<QN, std::vector<QN>, QCmp> open;
    gscore[sidx] = 0.0f;
    open.push(QN{h(sidx), sidx});

    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const float cost8[8] = {1.f, 1.f, 1.f, 1.f, 1.4142f, 1.4142f, 1.4142f, 1.4142f};

    std::vector<uint8_t> in_closed(N, 0);

    bool found = false;
    int expansions = 0;
    const int expansion_cap = N * 2;

    while (!open.empty() && expansions < expansion_cap) {
      const QN cur = open.top();
      open.pop();

      const int u = cur.idx;
      if (in_closed[u]) continue;
      in_closed[u] = 1;
      expansions++;

      if (u == gidx) {
        found = true;
        break;
      }

      auto [ux, uy] = to_xy(u, meta_);

      for (int k = 0; k < 8; k++) {
        const int vx = ux + dx8[k];
        const int vy = uy + dy8[k];
        if (!in_bounds(vx, vy, meta_)) continue;
        const int v = to_index(vx, vy, meta_);
        if (obstacle[v]) continue;

        const float tentative = gscore[u] + cost8[k];
        if (tentative < gscore[v]) {
          came_from[v] = u;
          gscore[v] = tentative;
          const float f = tentative + h(v);
          open.push(QN{f, v});
        }
      }
    }

    if (!found) return false;

    // reconstruct
    std::vector<int> idx_path;
    int cur = gidx;
    idx_path.reserve(2048);
    idx_path.push_back(cur);
    while (cur != sidx) {
      cur = came_from[cur];
      if (cur < 0) return false;
      idx_path.push_back(cur);
      if (idx_path.size() > static_cast<size_t>(N)) return false;
    }
    std::reverse(idx_path.begin(), idx_path.end());

    // simplify (remove near-collinear)
    idx_path = simplifyPath(idx_path, obstacle);

    // convert to poses
    rclcpp::Time stamp = this->now();
    out_path.reserve(idx_path.size());

    for (size_t i = 0; i < idx_path.size(); i++) {
      auto [cx, cy] = to_xy(idx_path[i], meta_);
      auto [mx, my] = cell_to_meters(cx, cy, meta_);

      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = map_frame_;
      ps.header.stamp = stamp;
      ps.pose.position.x = mx;
      ps.pose.position.y = my;
      ps.pose.position.z = 0.0;

      // yaw from segment direction
      double yaw = 0.0;
      if (i + 1 < idx_path.size()) {
        auto [nx, ny] = to_xy(idx_path[i + 1], meta_);
        yaw = std::atan2(static_cast<double>(ny - cy), static_cast<double>(nx - cx));
      } else if (i > 0) {
        auto [px, py] = to_xy(idx_path[i - 1], meta_);
        yaw = std::atan2(static_cast<double>(cy - py), static_cast<double>(cx - px));
      }
      ps.pose.orientation = yaw_to_quat(yaw);

      out_path.push_back(ps);
    }

    return !out_path.empty();
  }

  int nearestFree(int seed_idx, const std::vector<uint8_t> &obstacle) const {
    if (seed_idx < 0) return -1;
    const int N = meta_.width * meta_.height;
    std::vector<uint8_t> vis(N, 0);
    std::queue<int> q;
    q.push(seed_idx);
    vis[seed_idx] = 1;

    const int dx4[4] = {1, -1, 0, 0};
    const int dy4[4] = {0, 0, 1, -1};

    while (!q.empty()) {
      int cur = q.front();
      q.pop();
      if (!obstacle[cur]) return cur;

      auto [cx, cy] = to_xy(cur, meta_);
      for (int k = 0; k < 4; k++) {
        int nx = cx + dx4[k];
        int ny = cy + dy4[k];
        if (!in_bounds(nx, ny, meta_)) continue;
        int ni = to_index(nx, ny, meta_);
        if (vis[ni]) continue;
        vis[ni] = 1;
        q.push(ni);
      }
    }

    return -1;
  }

  std::vector<int> simplifyPath(const std::vector<int> &idx_path,
                                const std::vector<uint8_t> &obstacle) const {
    if (idx_path.size() < 3) return idx_path;

    // collinear prune first
    std::vector<int> pruned;
    pruned.reserve(idx_path.size());
    pruned.push_back(idx_path.front());

    for (size_t i = 1; i + 1 < idx_path.size(); i++) {
      auto [x0, y0] = to_xy(pruned.back(), meta_);
      auto [x1, y1] = to_xy(idx_path[i], meta_);
      auto [x2, y2] = to_xy(idx_path[i + 1], meta_);

      int dx1 = x1 - x0;
      int dy1 = y1 - y0;
      int dx2 = x2 - x1;
      int dy2 = y2 - y1;

      // reduce direction to -1/0/1
      dx1 = (dx1 > 0) ? 1 : (dx1 < 0 ? -1 : 0);
      dy1 = (dy1 > 0) ? 1 : (dy1 < 0 ? -1 : 0);
      dx2 = (dx2 > 0) ? 1 : (dx2 < 0 ? -1 : 0);
      dy2 = (dy2 > 0) ? 1 : (dy2 < 0 ? -1 : 0);

      if (dx1 == dx2 && dy1 == dy2) {
        continue;  // skip middle point
      }
      pruned.push_back(idx_path[i]);
    }
    pruned.push_back(idx_path.back());

    // line-of-sight shortcut
    std::vector<int> out;
    out.reserve(pruned.size());
    size_t i = 0;
    while (i < pruned.size()) {
      out.push_back(pruned[i]);
      size_t best = i + 1;
      for (size_t j = pruned.size() - 1; j > i; j--) {
        if (hasLineOfSight(pruned[i], pruned[j], obstacle)) {
          best = j;
          break;
        }
      }
      i = best;
      if (i == pruned.size() - 1) {
        out.push_back(pruned.back());
        break;
      }
    }

    // remove potential duplicates
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
  }

  bool hasLineOfSight(int a, int b, const std::vector<uint8_t> &obstacle) const {
    auto [x0, y0] = to_xy(a, meta_);
    auto [x1, y1] = to_xy(b, meta_);

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true) {
      if (!in_bounds(x, y, meta_)) return false;
      int idx = to_index(x, y, meta_);
      if (obstacle[idx]) return false;

      if (x == x1 && y == y1) break;
      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
    }

    return true;
  }

  void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> &poses) {
    nav_msgs::msg::Path p;
    p.header.frame_id = map_frame_;
    p.header.stamp = this->now();
    p.poses = poses;
    plan_pub_->publish(p);
  }

  // ===================== Control =====================
  void controlTick() {
    geometry_msgs::msg::Twist cmd;

    if (!goal_active_) {
      cmd_pub_->publish(cmd);
      return;
    }

    Pose2D robot{};
    if (!getRobotPose(robot)) {
      cmd_pub_->publish(cmd);
      return;
    }

    // stop if goal reached (in map)
    const double gx = goal_.pose.position.x;
    const double gy = goal_.pose.position.y;
    const double dx = gx - robot.x;
    const double dy = gy - robot.y;
    const double dist_to_goal = std::sqrt(dx * dx + dy * dy);

    if (dist_to_goal <= goal_tol_m_) {
      goal_active_ = false;
      have_path_ = false;
      need_replan_ = false;
      path_points_.clear();
      path_cursor_ = 0;
      publishPlan({});
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached.");
      return;
    }

    if (!have_path_ || path_points_.size() < 2) {
      // no path yet -> wait for planner
      cmd_pub_->publish(cmd);
      return;
    }

    // Scan-based safety:
    // - safe_dist_front: start reacting (request replan + slow down)
    // - stop_dist_front: hard-stop linear motion (emergency)
    // We still allow rotation-in-place for avoidance.

    // advance cursor to closest point
    updatePathCursor(robot);

    // pick lookahead point
    const auto target = pickLookahead(robot);
    if (!target.has_value()) {
      cmd_pub_->publish(cmd);
      return;
    }

    const double tx = target->first;
    const double ty = target->second;

    const double ang_to = std::atan2(ty - robot.y, tx - robot.x);
    const double err = norm_angle(ang_to - robot.yaw);

    double w = k_ang_ * err;
    w = std::clamp(w, -max_ang_, max_ang_);

    // Forward speed: allow moving while turning (pure-pursuit style) to avoid jitter.
    // Scale by cos(err) so we slow down when heading error is large.
    const double dist = std::sqrt((tx - robot.x) * (tx - robot.x) + (ty - robot.y) * (ty - robot.y));
    double v = std::min(max_lin_, k_lin_ * dist);
    const double turn_scale = std::max(0.0, std::cos(err));
    v *= turn_scale;

    // Keep a small minimum once we decide to move, otherwise the motor_interface may interpret as STOP.
    if (v > 0.0 && v < min_lin_move) v = min_lin_move;

    // If heading error is extremely large, rotate in place first.
    if (std::fabs(err) > angle_stop_rad_) {
      v = 0.0;
      if (std::fabs(w) > 0.0 && std::fabs(w) < min_ang_move) {
        w = (w >= 0.0) ? min_ang_move : -min_ang_move;
      }
    }

	    // Apply scan-based slow/stop logic AFTER computing the intended control.
	    // This avoids "always STOP" in normal indoor environments where a wall may be within
	    // safe_dist_front but not actually blocking the path direction.
	    if (have_scan_ && std::isfinite(front_min_dist_) && front_min_dist_ < safe_dist_front) {
	      // Always request a replan when something is inside the reaction distance.
	      need_replan_ = true;

	      // Only limit forward motion; allow rotation-in-place for avoidance.
	      if (v > 0.0) {
	        if (front_min_dist_ <= stop_dist_front) {
	          v = 0.0;
	        } else {
	          const double denom = std::max(1e-3, (safe_dist_front - stop_dist_front));
	          const double alpha = (front_min_dist_ - stop_dist_front) / denom; // 0..1
	          const double scale = std::clamp(alpha, 0.0, 1.0);
	          v *= scale;
	          if (v > 0.0 && v < min_lin_move) v = min_lin_move;
	        }
	      }
	    }

    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  void updatePathCursor(const Pose2D &robot) {
    // find nearest point within a window ahead
    size_t best_i = path_cursor_;
    double best_d2 = std::numeric_limits<double>::infinity();

    const size_t N = path_points_.size();
    const size_t start = std::min(path_cursor_, N - 1);
    const size_t end = std::min(N, start + 40);

    for (size_t i = start; i < end; i++) {
      const double px = path_points_[i].pose.position.x;
      const double py = path_points_[i].pose.position.y;
      const double dx = px - robot.x;
      const double dy = py - robot.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_i = i;
      }
    }

    path_cursor_ = best_i;
  }

  std::optional<std::pair<double, double>> pickLookahead(const Pose2D &robot) const {
    const size_t N = path_points_.size();
    if (N == 0) return std::nullopt;

    const double lh2 = lookahead_m_ * lookahead_m_;

    for (size_t i = path_cursor_; i < N; i++) {
      const double px = path_points_[i].pose.position.x;
      const double py = path_points_[i].pose.position.y;
      const double dx = px - robot.x;
      const double dy = py - robot.y;
      if ((dx * dx + dy * dy) >= lh2) {
        return std::make_pair(px, py);
      }
    }

    // fallback: last point
    const double px = path_points_.back().pose.position.x;
    const double py = path_points_.back().pose.position.y;
    return std::make_pair(px, py);
  }

  bool pathBlockedAhead() const {
    if (!have_map_ || !have_path_) return false;
    if (path_points_.empty()) return false;

    // check next few points
    const size_t N = path_points_.size();
    const size_t end = std::min(N, path_cursor_ + 25);

    // quick occupancy check on raw map only (inflation handled at plan time)
    for (size_t i = path_cursor_; i < end; i++) {
      const double mx = path_points_[i].pose.position.x;
      const double my = path_points_[i].pose.position.y;
      auto [cx, cy] = meters_to_cell(mx, my, meta_);
      if (!in_bounds(cx, cy, meta_)) continue;
      const int idx = to_index(cx, cy, meta_);
      const int8_t v = map_.data[idx];
      const bool occ = (v >= occupied_thresh_);
      if (occ) return true;
      if (use_scan_obstacles_) {
        auto it = scan_obstacles_.find(idx);
        if (it != scan_obstacles_.end()) {
          if ((this->now() - it->second).seconds() <= scan_obstacle_hold_sec) return true;
        }
      }
    }

    return false;
  }

  // ===================== TF & Scan helpers =====================
  bool getRobotPose(Pose2D &out) {
    try {
      auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      out.x = tf.transform.translation.x;
      out.y = tf.transform.translation.y;
      out.yaw = quat_to_yaw(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException &ex) {
      // NOTE: Avoid *_THROTTLE macros here to prevent const-clock issues on some builds.
      const auto now_t = this->now();
      if (!last_tf_warn_.nanoseconds() || (now_t - last_tf_warn_).seconds() > 2.0) {
        last_tf_warn_ = now_t;
        RCLCPP_WARN(this->get_logger(), "TF lookup failed (%s->%s): %s",
                    map_frame_.c_str(), base_frame_.c_str(), ex.what());
      }
      return false;
    }
  }

  static std::string normalize_frame(const std::string &f) {
    if (!f.empty() && f[0] == '/') return f.substr(1);
    return f;
  }

  void updateFrontMin(const sensor_msgs::msg::LaserScan &scan) {
    const double half = (front_half_angle_deg * M_PI) / 180.0;

    double best = std::numeric_limits<double>::infinity();
    double a = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); i++, a += scan.angle_increment) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r)) continue;
      if (r <= scan.range_min) continue;
      if (r >= scan.range_max) continue;
      if (std::fabs(a) > half) continue;
      best = std::min(best, static_cast<double>(r));
    }

    front_min_dist_ = std::isfinite(best) ? best : std::numeric_limits<double>::infinity();
  }

private:
  // topics
  std::string map_topic_;
  std::string goal_topic_;
  std::string scan_topic_;
  std::string cmd_topic_;
  std::string plan_topic_;

  // frames
  std::string map_frame_;
  std::string base_frame_;

  // parameters
  int occupied_thresh_{50};
  bool unknown_is_obstacle_{true};
  double inflation_radius_m_{0.18};

  double plan_rate_hz_{2.0};
  double control_rate_hz_{20.0};

  double lookahead_m_{0.35};
  double goal_tol_m_{0.12};
  double max_lin_{0.22};
  double max_ang_{0.8};
  double k_ang_{2.0};
  double k_lin_{0.9};
  double angle_stop_rad_{0.55};

	  bool use_scan_obstacles_{true};
	  double scan_obstacle_hold_sec{2.0};
	  double safe_dist_front{2.0};
	  double stop_dist_front{1.2};
	  double min_lin_move{0.12};
	  double min_ang_move{0.12};
	  double front_half_angle_deg{18.0};

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // state
  nav_msgs::msg::OccupancyGrid map_;
  bool have_map_{false};
  GridMeta meta_;

  geometry_msgs::msg::PoseStamped goal_;
  bool goal_active_{false};

  sensor_msgs::msg::LaserScan latest_scan_;
  bool have_scan_{false};
  double front_min_dist_{std::numeric_limits<double>::infinity()};

  bool need_replan_{false};
  bool have_path_{false};
  std::vector<geometry_msgs::msg::PoseStamped> path_points_;
  size_t path_cursor_{0};

  // log throttle (manual)
  rclcpp::Time last_tf_warn_{0, 0, RCL_ROS_TIME};

  // temporary obstacles (cell index -> last seen)
  std::unordered_map<int, rclcpp::Time> scan_obstacles_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridNav>());
  rclcpp::shutdown();
  return 0;
}
