#include "CYdLidar.h"
#include "ydlidar_driver.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"

#include <cmath>
#include <memory>
#include <string>

using namespace std;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ydlidar_s2pro");

  CYdLidar laser;

// -----------------------------------------------------
// STRING PARAMETERS
// -----------------------------------------------------
  std::string port =
    node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

  std::string ignore_array =
    node->declare_parameter<std::string>("ignore_array", "");
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

  std::string frame_id =
    node->declare_parameter<std::string>("frame_id", "laser_frame");

// -----------------------------------------------------
// INTEGER PARAMETERS
// -----------------------------------------------------
  int baudrate =
    node->declare_parameter<int>("baudrate", 115200);
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

  int lidar_type =
    node->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
  laser.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

  int device_type =
    node->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
  laser.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));

  int sample_rate =
    node->declare_parameter<int>("sample_rate", 4);
  laser.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));

  int abnormal_check =
    node->declare_parameter<int>("abnormal_check_count", 4);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &abnormal_check, sizeof(int));

// -----------------------------------------------------
// BOOLEAN PARAMETERS
// -----------------------------------------------------
  bool fixed_resolution =
    node->declare_parameter<bool>("resolution_fixed", true);
  laser.setlidaropt(LidarPropFixedResolution, &fixed_resolution, sizeof(bool));

  bool reversion =
    node->declare_parameter<bool>("reversion", false);
  laser.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));

  bool inverted =
    node->declare_parameter<bool>("inverted", true);
  laser.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));

  bool auto_reconnect =
    node->declare_parameter<bool>("auto_reconnect", true);
  laser.setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));

  bool is_single_channel =
    node->declare_parameter<bool>("isSingleChannel", true);
  laser.setlidaropt(LidarPropSingleChannel, &is_single_channel, sizeof(bool));

  bool intensity =
    node->declare_parameter<bool>("intensity", false);
  laser.setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));

  bool support_motor_dtr =
    node->declare_parameter<bool>("support_motor_dtr", true);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &support_motor_dtr, sizeof(bool));

  bool invalid_range_is_inf =
    node->declare_parameter<bool>("invalid_range_is_inf", false);

// -----------------------------------------------------
// FLOAT PARAMETERS
// -----------------------------------------------------
  float angle_max =
    node->declare_parameter<float>("angle_max", 180.0f);
  laser.setlidaropt(LidarPropMaxAngle, &angle_max, sizeof(float));

  float angle_min =
    node->declare_parameter<float>("angle_min", -180.0f);
  laser.setlidaropt(LidarPropMinAngle, &angle_min, sizeof(float));

  float range_max =
    node->declare_parameter<float>("range_max", 10.0f);
  laser.setlidaropt(LidarPropMaxRange, &range_max, sizeof(float));

  float range_min =
    node->declare_parameter<float>("range_min", 0.02f);
  laser.setlidaropt(LidarPropMinRange, &range_min, sizeof(float));

  float frequency =
    node->declare_parameter<float>("frequency", 10.0f);
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

// -----------------------------------------------------
// INITIALIZE & START LIDAR
// -----------------------------------------------------

  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Init Error: %s", laser.DescribeError());
  }

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

// -----------------------------------------------------
// SERVICE: STOP SCAN
// -----------------------------------------------------
  auto stop_scan_service =
    [&laser](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool
    {
      return laser.turnOff();
    };

  auto stop_service =
    node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_service);

// -----------------------------------------------------
// SERVICE: START SCAN
// -----------------------------------------------------
  auto start_scan_service =
    [&laser](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool
    {
      return laser.turnOn();
    };

  auto start_service =
    node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_service);

// -----------------------------------------------------
// MAIN LOOP: READ & PUBLISH SCAN
// -----------------------------------------------------

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    LaserScan scan;

    if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =
        scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;

      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;

      int size = (scan.config.max_angle - scan.config.min_angle) /
                 scan.config.angle_increment + 1;

      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);

      for (size_t i = 0; i < scan.points.size(); i++) {

        int index = std::ceil(
          (scan.points[i].angle - scan.config.min_angle) /
          scan.config.angle_increment);

        if (index >= 0 && index < size) {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }

      laser_pub->publish(*scan_msg);
    }
    else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

// -----------------------------------------------------
// SHUTDOWN
// -----------------------------------------------------
  RCLCPP_INFO(node->get_logger(), "Stopping LiDAR...");
  laser.turnOff();
  laser.disconnecting();

  rclcpp::shutdown();
  return 0;
}
