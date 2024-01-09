#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
    this->declare_parameter<std::double_t>("degrees", 360.0, degree_desc);

    // print to see if parameter Setting workedf
    this->get_parameter("obstacle", arg_obstacle);
    RCLCPP_INFO(this->get_logger(), "Obstacle parameter is: %f", arg_obstacle);

    this->get_parameter("degrees", arg_degrees);
    RCLCPP_INFO(this->get_logger(), "Degrees parameter is: %f", arg_degrees);

    // Init CallbackGroups
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // init subscription option
    rclcpp::SubscriptionOptions option1;
    option1.callback_group = callback_group_1;

    // // init laser subscription
    // laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "/scan", 1, std::bind(&RobotPatrol::scan_callback, this, _1),
    //     option1);

    // // init timer
    // timer_ = this->create_wall_timer(
    //     100ms, std::bind(&PreApproach::timer_callback, this),
    //     callback_group_2);

    // // init cmd_vel publisher
    // vel_pub_ =
    //     this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel",
    //     1);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    auto front_laser_reading = msg->ranges;

    cout << " length of range" << msg->ranges.size() << endl;

    cout << "right?" << msg->ranges[0] << endl;

    cout << "Center?" << msg->ranges[180] << endl;

    cout << "left?" << msg->ranges[359] << endl;

    // if (min_within_vision < avoid_dist) { // this finds the min within range
    //   linear_x = 0;
    //   if (last_direction > 0) {

    //     angular_z = -0.7;
    //     RCLCPP_INFO(this->get_logger(), "Avoiding obstacles: turning
    //     RIGHT!");

    //   } else {
    //     angular_z = 0.7;
    //     RCLCPP_INFO(this->get_logger(), "Avoiding obstacles: turning LEFT!");
    //   }
    //   // force sleep

    //   rclcpp::sleep_for(100ms);
    // } else {
    //   angular_z = direction_ / 2;
    //   // angular_z = 0.1;
    //   linear_x = 0.1;
    //   last_direction = direction_;
    // }

    // RCLCPP_INFO(this->get_logger(), "index is : '%d'", temp);
  }

  void timer_callback() {
    this->get_parameter("velocity", vel_parameter_);
    RCLCPP_INFO(this->get_logger(), "Velocity parameter is: %f",
                vel_parameter_);
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = vel_parameter_;
    vel_pub_->publish(message);
  }

private:
  // attributes
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;

  std::double_t vel_parameter_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  double speed_linear_x, speed_angular_z;
  double arg_obstacle, arg_degrees;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}