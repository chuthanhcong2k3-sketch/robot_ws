
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>
#include <string>

class MotorInterface : public rclcpp::Node
{
public:
    MotorInterface() : Node("motor_interface")
    {
        this->declare_parameter<std::string>("port", "/dev/stm32");
        port_ = this->get_parameter("port").as_string();

        // Hysteresis to avoid STOP/MOVE jitter when multiple publishers or small commands exist
        start_lin_ = this->declare_parameter<double>("start_lin", 0.06);
        stop_lin_  = this->declare_parameter<double>("stop_lin", 0.03);
        start_ang_ = this->declare_parameter<double>("start_ang", 0.10);
        stop_ang_  = this->declare_parameter<double>("stop_ang", 0.05);
        send_period_ms_ = this->declare_parameter<int>("send_period_ms", 50);

        openSerial();

        // ---- SUBSCRIBE CMD_VEL ----
        sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&MotorInterface::cb_cmd_vel, this, std::placeholders::_1));

        // ---- SUBSCRIBE ARM ----
        sub_arm_ = this->create_subscription<std_msgs::msg::String>(
            "arm_cmd", 10,
            std::bind(&MotorInterface::cb_arm_cmd, this, std::placeholders::_1));

        // ---- SUBSCRIBE GRIPPER ----
        sub_grip_ = this->create_subscription<std_msgs::msg::String>(
            "gripper_cmd", 10,
            std::bind(&MotorInterface::cb_grip_cmd, this, std::placeholders::_1));

        // ---- SUBSCRIBE MANUAL (WEB CONTROL) ----
        sub_manual_ = this->create_subscription<std_msgs::msg::String>(
            "manual_cmd", 10,
            std::bind(&MotorInterface::cb_manual_cmd, this, std::placeholders::_1));

        last_drive_cmd_ = "STOP";
        last_send_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "MotorInterface started (TEXT MODE). Port=%s, start_lin=%.2f stop_lin=%.2f start_ang=%.2f stop_ang=%.2f",
                    port_.c_str(), start_lin_, stop_lin_, start_ang_, stop_ang_);
    }

private:
    int serial_fd_ = -1;
    std::string port_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_arm_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_grip_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_manual_;

    double start_lin_{0.06};
    double stop_lin_{0.03};
    double start_ang_{0.10};
    double stop_ang_{0.05};
    int send_period_ms_{50};

    std::string last_drive_cmd_{"STOP"};
    rclcpp::Time last_send_time_{0, 0, RCL_ROS_TIME};

    // =======================================================
    // OPEN SERIAL
    // =======================================================
    void openSerial()
    {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "FAILED TO OPEN %s", port_.c_str());
            return;
        }

        struct termios tty{};
        tcgetattr(serial_fd_, &tty);

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        tcsetattr(serial_fd_, TCSANOW, &tty);

        RCLCPP_INFO(this->get_logger(), "Serial opened OK!");
    }

    // =======================================================
    // SEND STRING TO STM32
    // =======================================================
    void sendLine(const std::string &s)
    {
        if (serial_fd_ < 0) return;
        std::string msg = s + "\n";
        (void)write(serial_fd_, msg.c_str(), msg.size());
    }

    // =======================================================
    // Decide command with hysteresis to prevent jitter
    // =======================================================
    std::string decideDriveCmd(float vx, float wz)
    {
        const float avx = std::fabs(vx);
        const float awz = std::fabs(wz);

        auto want_fwd  = vx > 0.0f;
        auto want_left = wz > 0.0f;

        const bool last_stop = (last_drive_cmd_ == "STOP");
        const bool last_lin  = (last_drive_cmd_ == "MOVE_FWD" || last_drive_cmd_ == "MOVE_BACK");
        const bool last_turn = (last_drive_cmd_ == "TURN_LEFT" || last_drive_cmd_ == "TURN_RIGHT");

        // Start conditions
        const bool start_lin = (avx >= start_lin_);
        const bool start_ang = (awz >= start_ang_);

        // Stop conditions (hysteresis)
        const bool stop_all  = (avx <= stop_lin_ && awz <= stop_ang_);

        if (last_stop) {
            if (start_lin) return want_fwd ? "MOVE_FWD" : "MOVE_BACK";
            if (start_ang) return want_left ? "TURN_LEFT" : "TURN_RIGHT";
            return "STOP";
        }

        // If currently moving linearly
        if (last_lin) {
            if (stop_all) return "STOP";
            if (start_lin) return want_fwd ? "MOVE_FWD" : "MOVE_BACK";
            if (start_ang) return want_left ? "TURN_LEFT" : "TURN_RIGHT";
            // Keep previous if command is weak/noisy
            return last_drive_cmd_;
        }

        // If currently turning
        if (last_turn) {
            if (stop_all) return "STOP";
            if (start_ang) return want_left ? "TURN_LEFT" : "TURN_RIGHT";
            if (start_lin) return want_fwd ? "MOVE_FWD" : "MOVE_BACK";
            return last_drive_cmd_;
        }

        // Fallback
        if (start_lin) return want_fwd ? "MOVE_FWD" : "MOVE_BACK";
        if (start_ang) return want_left ? "TURN_LEFT" : "TURN_RIGHT";
        return "STOP";
    }

    // =======================================================
    // CMD_VEL → TEXT COMMAND
    // =======================================================
    void cb_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const float vx = static_cast<float>(msg->linear.x);
        const float wz = static_cast<float>(msg->angular.z);

        const std::string cmd = decideDriveCmd(vx, wz);

        // Rate-limit sending to serial to reduce spam & jitter.
        const auto now_t = this->now();
        const auto elapsed_ns = (now_t - last_send_time_).nanoseconds();
        const long period_ns = static_cast<long>(send_period_ms_) * 1000000L;

        bool should_send = false;
        if (cmd != last_drive_cmd_) {
            should_send = true;
        } else if (cmd != "STOP" && elapsed_ns >= period_ns) {
            // keep-alive for motion commands
            should_send = true;
        }

        if (should_send) {
            sendLine(cmd);
            last_send_time_ = now_t;
            last_drive_cmd_ = cmd;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                            "cmd_vel vx=%.3f wz=%.3f -> %s", vx, wz, cmd.c_str());
    }

    // =======================================================
    // ARM
    // =======================================================
    void cb_arm_cmd(const std_msgs::msg::String::SharedPtr msg)
    {
        sendLine(msg->data);
    }

    // =======================================================
    // GRIPPER
    // =======================================================
    void cb_grip_cmd(const std_msgs::msg::String::SharedPtr msg)
    {
        sendLine(msg->data);
    }

    // =======================================================
    // MANUAL CONTROL FROM WEB
    // =======================================================
    void cb_manual_cmd(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string s = msg->data;

        // Fix đảo trái/phải chỉ cho MANUAL
        if (s == "TURN_LEFT") s = "TURN_RIGHT";
        else if (s == "TURN_RIGHT") s = "TURN_LEFT";

        sendLine(s);
        last_drive_cmd_ = s;           // keep hysteresis consistent
        last_send_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "MANUAL -> %s (sent: %s)", msg->data.c_str(), s.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorInterface>());
    rclcpp::shutdown();
    return 0;
}
