#ifndef COMPOSITION__PREAPPROACH_COMPONENT_HPP_
#define COMPOSITION__PREAPPROACH_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC // what does
      explicit PreApproach(const rclcpp::NodeOptions &options);

private:
  // arg params (Hard Coded)
  double arg_obstacle;
  int arg_degrees;

  // speed
  double MAX_ANGULAR_SPEED;
  float speed_linear_x, speed_angular_z;

  // pre approach params: wall flag;
  bool wall_reached = false;
  bool finished_rot = false;

  float cur_yaw;
  float target_yaw;

  void move();
  void rotate();
  void set_target_yaw();

  // callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

} // namespace my_components

#endif // COMPOSITION__PREAPPROACH_COMPONENT_HPP_