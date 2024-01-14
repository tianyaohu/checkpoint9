#include "my_components/pre_approach.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std;

namespace my_components {
PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("odom_subscriber", options) {

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 5,
      std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

  // QOS profile
  rclcpp::QoS qos_profile_subscriber(1);
  qos_profile_subscriber.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // init laser subscription with QOS
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos_profile_subscriber,
      std::bind(&PreApproach::scan_callback, this, _1));

  // init cmd_vel publisher
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  // init args
  arg_obstacle = 0.3;
  arg_degrees = -90;

  // set initial speed
  speed_linear_x = 0.4;
  speed_angular_z = 0.0;

  MAX_ANGULAR_SPEED = 0.7;
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // if wall not reached or rotation is finished do nothing
  const float q_x = msg->pose.pose.orientation.x;
  const float q_y = msg->pose.pose.orientation.y;
  const float q_z = msg->pose.pose.orientation.z;
  const float q_w = msg->pose.pose.orientation.w;

  // Yaw (z-axis rotation)
  float sinYaw = 2.0f * (q_w * q_z + q_x * q_y);
  float cosYaw = 1.0f - 2.0f * (q_y * q_y + q_z * q_z);
  cur_yaw = std::atan2(sinYaw, cosYaw);
  // if wall is reached and rotation is not done
  if (wall_reached && finished_rot == false) {
    rotate();
  }
}

void PreApproach::rotate() {
  // calc turn delta
  double turn_delta = cur_yaw - target_yaw;
  // normalize turn delta
  if (abs(turn_delta) > M_PI) {
    turn_delta =
        turn_delta > 0 ? -2 * M_PI + turn_delta : 2 * M_PI + turn_delta;
  }
  // set angular speed
  speed_angular_z = min(MAX_ANGULAR_SPEED, turn_delta / -1);

  RCLCPP_INFO(this->get_logger(), "turn_delta? %f", turn_delta);
  RCLCPP_INFO(this->get_logger(), "cur_yaw? %f", cur_yaw);
  RCLCPP_INFO(this->get_logger(), "finished_rot? %s",
              finished_rot ? "True" : "False");

  if (turn_delta < 0.05) {
    speed_angular_z = 0;
    finished_rot = true;
    RCLCPP_INFO(this->get_logger(), "Final Yaw upon finishing? %f", cur_yaw);
  } else {

    move();
  }
  // logger seperator
  RCLCPP_INFO(this->get_logger(), "------------------------------------------");
}

void PreApproach::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // get the laser reading of front
  float front_dist = msg->ranges[540];

  if (wall_reached == false) {
    RCLCPP_INFO(this->get_logger(), "front obstacle distance? %f", front_dist);
  }

  // check if obstacle is within distance, and if wall_reached flag is raised
  if (front_dist <= arg_obstacle && wall_reached == false) {
    speed_linear_x = 0;
    speed_angular_z = 0;
    // init target yaw
    set_target_yaw();
    wall_reached = true;
  }

  // only triggers if not finished rotation
  if (!finished_rot) {
    move();
  }
}

// set target_yaw
void PreApproach::set_target_yaw() {
  // get current yaw
  target_yaw = cur_yaw + (arg_degrees % 360) * M_PI / 180;
}

void PreApproach::move() {
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = speed_linear_x;
  message.angular.z = speed_angular_z;
  this->vel_pub_->publish(message);

  RCLCPP_INFO(this->get_logger(), "speed linear x? %f", speed_linear_x);
  RCLCPP_INFO(this->get_logger(), "speed angular z %f", speed_angular_z);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)