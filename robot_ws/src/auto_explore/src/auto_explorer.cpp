#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

class AutoExplorer : public rclcpp::Node {
public:
    AutoExplorer() : Node("auto_explorer")
    {
        this->declare_parameter("forward_speed", 0.20);
        this->declare_parameter("turn_speed", 0.45);
        this->declare_parameter("safe_dist", 2.0);
        this->declare_parameter("side_angle", 60.0);

        // ✅ ADD: góc trung tâm “phía trước robot” theo LIDAR (CÁCH 3)
        // Ví dụ front bị ngược thì set 180
        this->declare_parameter("front_angle_center_deg", 0.0);

        forward_speed_ = this->get_parameter("forward_speed").as_double();
        turn_speed_    = this->get_parameter("turn_speed").as_double();
        safe_dist_     = this->get_parameter("safe_dist").as_double();
        side_angle_    = this->get_parameter("side_angle").as_double();
        front_angle_center_deg_ = this->get_parameter("front_angle_center_deg").as_double();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&AutoExplorer::scan_callback, this, std::placeholders::_1)
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(),
                    "Auto Explorer started! front_center=%.1f deg",
                    front_angle_center_deg_);
    }

private:
    double forward_speed_, turn_speed_, safe_dist_, side_angle_;
    double front_angle_center_deg_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    static inline double ang_norm(double a) {
        // normalize rad -> [-pi, pi]
        return std::atan2(std::sin(a), std::cos(a));
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // ✅ CÁCH 3: offset mọi góc theo front_angle_center_deg_
    double front = get_region_distance(
       msg,
       front_angle_center_deg_ - 10.0,
       front_angle_center_deg_ + 10.0
    );

    double left = get_region_distance(
       msg,
       front_angle_center_deg_ + 40.0,
       front_angle_center_deg_ + 70.0
    );

    double right = get_region_distance(
       msg,
       front_angle_center_deg_ - 70.0,
       front_angle_center_deg_ - 40.0
    );

        geometry_msgs::msg::Twist cmd;

        if (front < safe_dist_) {
            // Bị chặn → chọn bên nào xa hơn để né
            if (left > right) {
                cmd.angular.z = -turn_speed_;
            } else {
                cmd.angular.z = turn_speed_;
            }
        } else {
            // Thoáng → đi thẳng
            cmd.linear.x = forward_speed_;
        }

        cmd_pub_->publish(cmd);
    }

    double get_region_distance(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                               double deg_start, double deg_end)
    {
        if (msg->ranges.empty()) return msg->range_max;

        // đảm bảo start <= end
        if (deg_end < deg_start) std::swap(deg_start, deg_end);

        const double rad_start = deg_start * M_PI / 180.0;
        const double rad_end   = deg_end   * M_PI / 180.0;

        double min_dist = std::numeric_limits<double>::infinity();

        // ✅ Duyệt theo angle thật + normalize, giống cách 3 của bạn bên follow
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const double angle = msg->angle_min + (double)i * msg->angle_increment;

            // check angle thuộc [rad_start, rad_end] theo kiểu wrap [-pi,pi]
            // Ta convert cả 2 về diff so với mid để tránh lỗi khi vùng cắt qua -pi/pi
            const double mid = ang_norm((rad_start + rad_end) * 0.5);
            const double half = std::abs(ang_norm(rad_end - rad_start)) * 0.5;

            const double diff = ang_norm(angle - mid);
            if (std::abs(diff) > half) continue;

            const float r = msg->ranges[i];

            // ✅ FIX QUAN TRỌNG: lọc rác của lidar (0.0, invalid, out of range)
            if (!std::isfinite(r)) continue;
            if (r <= msg->range_min) continue; // gồm cả 0.0
            if (r >= msg->range_max) continue;

            min_dist = std::min(min_dist, (double)r);
        }

        // nếu không có tia hợp lệ -> trả range_max
        if (!std::isfinite(min_dist)) return msg->range_max;
        return min_dist;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoExplorer>());
    rclcpp::shutdown();
    return 0;
}
