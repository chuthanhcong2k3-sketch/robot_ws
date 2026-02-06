#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>   // ✅ ADD: debug distance
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <deque>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>

static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

class TargetNavNode : public rclcpp::Node {
public:
  TargetNavNode() : Node("target_nav_node") {
    // ===== Params =====
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera_follow/image_raw");
    scan_topic_  = this->declare_parameter<std::string>("scan_topic",  "/scan");
    cmd_topic_   = this->declare_parameter<std::string>("cmd_topic",   "/cmd_vel");

    min_area_   = this->declare_parameter<int>("min_area", 1200);
    kp_yaw_     = this->declare_parameter<double>("kp_yaw", 0.8);
    search_wz_  = this->declare_parameter<double>("search_wz", 0.6);
    fwd_vx_     = this->declare_parameter<double>("fwd_vx", 0.25);

    center_tol_ = this->declare_parameter<double>("center_tol", 0.08);

    // ✅ DỪNG 50cm – SIÊU NHẠY (giữ như bạn đang set)
    stop_dist_  = this->declare_parameter<double>("stop_dist", 0.50);
    stop_band_  = this->declare_parameter<double>("stop_band", 0.02);

    arrived_confirm_frames_ =
      this->declare_parameter<int>("arrived_confirm_frames", 2);

    resume_dist_ =
      this->declare_parameter<double>("resume_dist", 0.55);

    // sector half-width
    front_angle_deg_ =
      this->declare_parameter<double>("front_angle_deg", 10.0);

    // ✅ ADD: góc trung tâm “phía trước robot” theo LIDAR (CỰC QUAN TRỌNG)
    // Bạn sẽ chỉnh cái này để match hướng thật:
    // 0   = sau lidar 
    // 90  = phải
    // -90 = trái
    // 180/-180 = trước lidar
    front_angle_center_deg_ =
      this->declare_parameter<double>("front_angle_center_deg", 180.0);

    // ===== Pub/Sub =====
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    pub_detected_ = this->create_publisher<std_msgs::msg::Bool>("/target_detected", 10);
    pub_near_     = this->create_publisher<std_msgs::msg::Bool>("/near_target", 10);

    // ✅ ADD: debug lidar distance
    pub_front_dist_ = this->create_publisher<std_msgs::msg::Float32>("/front_dist", 10);

    sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TargetNavNode::onImage, this, std::placeholders::_1));

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&TargetNavNode::onScan, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "target_nav_node started. STOP=%.2fm, sector=±%.1fdeg, front_center=%.1fdeg",
      stop_dist_, front_angle_deg_, front_angle_center_deg_);
  }

private:
  // ===== Scan =====
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
    last_front_dist_ = getFrontDistance();

    // ✅ publish debug
    std_msgs::msg::Float32 f;
    f.data = std::isfinite(last_front_dist_) ? (float)last_front_dist_ : -1.0f;
    pub_front_dist_->publish(f);
  }

  // ===== Image =====
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (...) {
      return;
    }

    const int w = bgr.cols;

    bool found;
    int cx, cy;
    double area;
    std::tie(found, cx, cy, area) = detectRed(bgr);

    // ===== UI detected =====
    camera_detected_ = found;
    std_msgs::msg::Bool dmsg;
    dmsg.data = camera_detected_;
    pub_detected_->publish(dmsg);

    // ===== SEARCH =====
    if (!camera_detected_) {
      stop_latch_ = false;
      arrived_count_ = 0;

      std_msgs::msg::Bool nmsg;
      nmsg.data = false;
      pub_near_->publish(nmsg);

      publishCmd(0.0, search_wz_);
      return;
    }

    // ===== TRACK =====
    const double err = ((w * 0.5) - cx) / (w * 0.5);
    double wz = -kp_yaw_ * err;

    if (std::abs(wz) < 0.12 && std::abs(err) > center_tol_) {
      wz = (wz > 0) ? 0.12 : -0.12;
    }

    const bool lidar_valid = std::isfinite(last_front_dist_);

    // ====================================================
    // ✅ STOP LOGIC: chỉ dựa vào “trước mặt” + khoảng cách
    // ====================================================
    // Dùng stop_dist_ + stop_band_ để dễ dừng hơn (siêu nhạy)
    const bool front_close =
      lidar_valid && (last_front_dist_ <= (stop_dist_ + stop_band_));

    if (front_close) arrived_count_++;
    else arrived_count_ = 0;

    const bool should_stop_now =
      arrived_count_ >= arrived_confirm_frames_;

    if (should_stop_now) {
      stop_latch_ = true;

      std_msgs::msg::Bool nmsg;
      nmsg.data = true;
      pub_near_->publish(nmsg);

      publishCmd(0.0, 0.0);
      return;
    }

    // ===== GIỮ STOP CHO TỚI KHI THẬT SỰ XA =====
    if (stop_latch_) {
      if (lidar_valid && last_front_dist_ > resume_dist_) {
        stop_latch_ = false;
        arrived_count_ = 0;
      } else {
        std_msgs::msg::Bool nmsg;
        nmsg.data = true;
        pub_near_->publish(nmsg);

        publishCmd(0.0, 0.0);
        return;
      }
    }

    // ===== CHƯA TỚI =====
    std_msgs::msg::Bool nmsg;
    nmsg.data = false;
    pub_near_->publish(nmsg);

    double vx = (std::abs(err) < 0.35) ? fwd_vx_ : 0.0;
    publishCmd(vx, wz);
  }

  // ===== RED DETECTION =====
  std::tuple<bool,int,int,double> detectRed(const cv::Mat &bgr) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat m1, m2, mask;
    cv::inRange(hsv, cv::Scalar(0,120,70),   cv::Scalar(10,255,255),  m1);
    cv::inRange(hsv, cv::Scalar(170,120,70), cv::Scalar(180,255,255), m2);
    cv::bitwise_or(m1, m2, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return {false,0,0,0};

    auto it = std::max_element(contours.begin(), contours.end(),
      [](auto &a, auto &b){ return cv::contourArea(a) < cv::contourArea(b); });

    double area = cv::contourArea(*it);
    if (area < min_area_) return {false,0,0,area};

    auto m = cv::moments(*it);
    if (m.m00 == 0) return {false,0,0,area};

    return {true, int(m.m10/m.m00), int(m.m01/m.m00), area};
  }

  // ===== FRONT LIDAR (FIX CHÍNH) =====
  double getFrontDistance() {
    if (!latest_scan_) return std::numeric_limits<double>::quiet_NaN();
    const auto &s = *latest_scan_;
    if (s.ranges.empty()) return std::numeric_limits<double>::quiet_NaN();

    const double half = front_angle_deg_ * M_PI / 180.0;
    const double center = front_angle_center_deg_ * M_PI / 180.0;

    auto ang_norm = [&](double a) -> double {
      // normalize to [-pi, pi]
      return std::atan2(std::sin(a), std::cos(a));
    };

    double best = std::numeric_limits<double>::infinity();

    // ✅ Duyệt theo angle thật, không phụ thuộc “0 rad có đúng là phía trước không”
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      const double angle = s.angle_min + (double)i * s.angle_increment;
      const double diff = ang_norm(angle - center);

      if (std::abs(diff) > half) continue;

      const float r = s.ranges[i];

      // ✅ lọc invalid như bạn đã thấy: 0.0 rất nhiều
      if (!std::isfinite(r)) continue;
      if (r <= s.range_min) continue;  // includes 0.0
      if (r >= s.range_max) continue;

      best = std::min(best, (double)r);
    }

    return std::isfinite(best) ? best : std::numeric_limits<double>::quiet_NaN();
  }

  void publishCmd(double vx, double wz) {
    geometry_msgs::msg::Twist t;
    t.linear.x = vx;
    t.angular.z = wz;
    pub_cmd_->publish(t);
  }

private:
  std::string image_topic_, scan_topic_, cmd_topic_;

  int min_area_;
  double kp_yaw_, search_wz_, fwd_vx_;
  double center_tol_, stop_dist_, stop_band_;
  double resume_dist_;
  int arrived_confirm_frames_;
  double front_angle_deg_;

  // ✅ ADD
  double front_angle_center_deg_;

  bool camera_detected_ = false;
  bool stop_latch_ = false;
  int arrived_count_ = 0;
  double last_front_dist_ = std::numeric_limits<double>::quiet_NaN();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_detected_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_near_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_front_dist_; // ✅ ADD

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetNavNode>());
  rclcpp::shutdown();
  return 0;
}
