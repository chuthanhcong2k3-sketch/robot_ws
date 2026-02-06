#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

class GoToGoal : public rclcpp::Node
{
public:
    GoToGoal()
    : Node("goto_goal")
    {
        // ---- Parameters ----
        this->declare_parameter<double>("max_lin_vel", 0.20);
        this->declare_parameter<double>("max_ang_vel", 0.6);
        this->declare_parameter<double>("k_lin", 0.8);
        this->declare_parameter<double>("k_ang", 1.5);
        this->declare_parameter<double>("goal_tol", 0.10);       // 10 cm
        this->declare_parameter<double>("angle_tol", 0.25);      // ~14 deg
        this->declare_parameter<double>("safe_dist_front", 2.0); // tránh chướng ngại

        this->get_parameter("max_lin_vel", max_lin_vel_);
        this->get_parameter("max_ang_vel", max_ang_vel_);
        this->get_parameter("k_lin",       k_lin_);
        this->get_parameter("k_ang",       k_ang_);
        this->get_parameter("goal_tol",    goal_tol_);
        this->get_parameter("angle_tol",   angle_tol_);
        this->get_parameter("safe_dist_front", safe_dist_front_);

        // ---- TF ----
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ---- ROS I/O ----
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&GoToGoal::goalCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&GoToGoal::scanCallback, this, std::placeholders::_1));

        // Loop control 20 Hz
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GoToGoal::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "goto_goal node started");
    }

private:
    // ---------- Callbacks ----------
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // assume frame_id == "map"
        latest_goal_ = *msg;
        has_goal_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "New goal: (%.2f, %.2f) in %s",
                    msg->pose.position.x,
                    msg->pose.position.y,
                    msg->header.frame_id.c_str());
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // tìm khoảng cách nhỏ nhất ở phía trước (±30deg)
        double min_front = std::numeric_limits<double>::infinity();

        int n = msg->ranges.size();
        if (n == 0) return;

        double angle = msg->angle_min;
        for (int i = 0; i < n; ++i, angle += msg->angle_increment) {
            if (std::isnan(msg->ranges[i]) || std::isinf(msg->ranges[i])) continue;

            // tiền phương ±30deg
            if (std::fabs(angle) < M_PI/6.0) {
                if (msg->ranges[i] < min_front) {
                    min_front = msg->ranges[i];
                }
            }
        }

        min_front_dist_ = min_front;
        have_scan_ = true;
    }

    // ---------- Main control loop ----------
    void controlLoop()
    {
        geometry_msgs::msg::Twist cmd;

        if (!has_goal_) {
            // không có goal -> dừng
            cmd_pub_->publish(cmd);
            return;
        }

        // Lấy pose hiện tại của robot trong frame "map"
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                 2000, "TF lookup failed: %s", ex.what());
            cmd_pub_->publish(geometry_msgs::msg::Twist()); // dừng
            return;
        }

        double rx = tf.transform.translation.x;
        double ry = tf.transform.translation.y;

        // yaw của robot
        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // vector tới goal
        double gx = latest_goal_.pose.position.x;
        double gy = latest_goal_.pose.position.y;

        double dx = gx - rx;
        double dy = gy - ry;
        double dist = std::sqrt(dx*dx + dy*dy);

        // Nếu đã tới gần
        if (dist < goal_tol_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                                 2000, "Goal reached (dist=%.3f)", dist);
            has_goal_ = false;
            cmd_pub_->publish(geometry_msgs::msg::Twist()); // dừng
            return;
        }

        // góc từ robot → goal (trong map)
        double target_yaw = std::atan2(dy, dx);
        double err_yaw = normalizeAngle(target_yaw - yaw);

        // ---- Obstacle avoidance (simple) ----
        bool block_front = have_scan_ &&
                           min_front_dist_ < safe_dist_front_;

        double v = 0.0;
        double w = 0.0;

        if (block_front) {
            // phía trước có vật cản quá gần → chỉ quay tại chỗ
            v = 0.0;
            w = 0.4; // quay trái nhẹ
            RCLCPP_DEBUG(this->get_logger(),
                         "Obstacle front: %.2f m → rotate in place", min_front_dist_);
        } else {
            // ---- Điều khiển đến goal ----
            if (std::fabs(err_yaw) > angle_tol_) {
                // lệch nhiều → ưu tiên quay
                v = 0.0;
                w = k_ang_ * err_yaw;
            } else {
                // hướng tương đối ổn → vừa đi vừa chỉnh
                v = k_lin_ * dist;
                w = k_ang_ * err_yaw;
            }
        }

        // saturate
        if (v >  max_lin_vel_) v =  max_lin_vel_;
        if (v < -max_lin_vel_) v = -max_lin_vel_;
        if (w >  max_ang_vel_) w =  max_ang_vel_;
        if (w < -max_ang_vel_) w = -max_ang_vel_;

        cmd.linear.x  = v;
        cmd.angular.z = w;

        cmd_pub_->publish(cmd);
    }

    static double normalizeAngle(double a)
    {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // ---- Members ----
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PoseStamped latest_goal_;
    bool has_goal_{false};

    bool have_scan_{false};
    double min_front_dist_{std::numeric_limits<double>::infinity()};

    double max_lin_vel_;
    double max_ang_vel_;
    double k_lin_;
    double k_ang_;
    double goal_tol_;
    double angle_tol_;
    double safe_dist_front_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToGoal>());
    rclcpp::shutdown();
    return 0;
}
