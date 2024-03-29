#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace std;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node") {
    // parameter descriptions
    auto obstacle_desc = rcl_interfaces::msg::ParameterDescriptor{};
    auto degree_desc = rcl_interfaces::msg::ParameterDescriptor{};

    // param description details
    obstacle_desc.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";
    degree_desc.description =
        "Number of degrees for the rotation of the robot after stopping";

    // declearing parameters
    this->declare_parameter<std::double_t>("obstacle", 2.0, obstacle_desc);
    this->declare_parameter<int>("degrees", 360, degree_desc);

    // print to see if parameter Setting workedf
    this->get_parameter("obstacle", arg_obstacle);
    RCLCPP_INFO(this->get_logger(), "Obstacle parameter is: %f", arg_obstacle);

    this->get_parameter("degrees", arg_degrees);
    RCLCPP_INFO(this->get_logger(), "Degrees parameter is: %d", arg_degrees);

    // Init CallbackGroups
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // init subscription options
    rclcpp::SubscriptionOptions option1;
    option1.callback_group = callback_group_1;
    rclcpp::SubscriptionOptions option2;
    option1.callback_group = callback_group_2;

    // QOS profile
    rclcpp::QoS qos_profile_subscriber(1);
    qos_profile_subscriber.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // init laser subscription with QOS
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile_subscriber,
        std::bind(&PreApproach::scan_callback, this, _1), option1);

    // init odom sub
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&PreApproach::odom_callback, this, _1), option2);

    // init timer
    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::timer_callback, this), callback_group_2);

    // init cmd_vel publisher
    vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    // set initial speed
    speed_linear_x = 0.2;
    speed_angular_z = 0.0;

    MAX_ANGULAR_SPEED = 0.5;
  }

private:
  // attributes
  // call back groups
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::CallbackGroup::SharedPtr callback_group_3;

  // sub and pubs
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  double MAX_ANGULAR_SPEED;
  float speed_linear_x, speed_angular_z;

  // arg params
  double arg_obstacle;
  int arg_degrees;
  bool wall_reached;

  // rot vars
  bool finished_rot;
  bool target_inited;
  double target_yaw;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // get the laser reading of front
    float front_dist = msg->ranges[540];

    if (wall_reached == false) {
      RCLCPP_INFO(this->get_logger(), "front obstacle distance? %f",
                  front_dist);
    }

    // check if obstacle is within distance, and if wall_reached flag is raised
    if (front_dist <= arg_obstacle && wall_reached == false) {
      speed_linear_x = 0;
      speed_angular_z = 0;
      // set wall reached flag
      wall_reached = true;
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // if wall is reached and rotation is not performed
    if (wall_reached && finished_rot == false) {
      // if wall not reached or rotation is finished do nothing
      const float q_x = msg->pose.pose.orientation.x;
      const float q_y = msg->pose.pose.orientation.y;
      const float q_z = msg->pose.pose.orientation.z;
      const float q_w = msg->pose.pose.orientation.w;

      // Yaw (z-axis rotation)
      float sinYaw = 2.0f * (q_w * q_z + q_x * q_y);
      float cosYaw = 1.0f - 2.0f * (q_y * q_y + q_z * q_z);
      float cur_yaw = std::atan2(sinYaw, cosYaw);

      // if target_yaw is not set, set it
      if (!target_inited) {
        // set cur_yaw
        double temp = (arg_degrees % 360) * M_PI / 180;
        target_yaw = cur_yaw + (arg_degrees % 360) * M_PI / 180;
        RCLCPP_INFO(this->get_logger(), "target_yaw? %f", target_yaw);
        RCLCPP_INFO(this->get_logger(), "temp %f", temp);

        if (abs(target_yaw) > M_PI) {
          target_yaw =
              target_yaw > 0 ? -2 * M_PI + target_yaw : 2 * M_PI + target_yaw;
        }
        // set flag
        target_inited = true;
        // if target is inited start turning
      } else {
        double turn_delta = cur_yaw - target_yaw;
        if (abs(turn_delta) > M_PI) {
          turn_delta =
              turn_delta > 0 ? -2 * M_PI + turn_delta : 2 * M_PI + turn_delta;
        }
        // set angular speed
        speed_angular_z = min(MAX_ANGULAR_SPEED, turn_delta / -3);

        RCLCPP_INFO(this->get_logger(), "turn_delta? %f", turn_delta);
        RCLCPP_INFO(this->get_logger(), "cur_yaw? %f", cur_yaw);
        RCLCPP_INFO(this->get_logger(), "finished_rot? %s",
                    finished_rot ? "True" : "False");

        if (turn_delta < 0.05) {
          speed_angular_z = 0;
          finished_rot = true;
          RCLCPP_INFO(this->get_logger(), "Final Yaw upon finishing? %f",
                      cur_yaw);
        }
      }
    }
  }

  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = speed_linear_x;
    message.angular.z = speed_angular_z;
    vel_pub_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // init node
  std::shared_ptr<PreApproach> node = std::make_shared<PreApproach>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}