#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
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

    // init subscription option
    rclcpp::SubscriptionOptions option1;
    option1.callback_group = callback_group_1;

    // QOS profile
    rclcpp::QoS qos_profile_subscriber(1);
    qos_profile_subscriber.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // init laser subscription with QOS
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile_subscriber,
        std::bind(&PreApproach::scan_callback, this, _1), option1);

    // init timer
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&PreApproach::timer_callback, this),
        callback_group_2);

    // init cmd_vel publisher
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    speed_linear_x = 0.2;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "length of range: %ld", msg->ranges.size());

    RCLCPP_INFO(this->get_logger(), "0? %f", msg->ranges[0]);
    RCLCPP_INFO(this->get_logger(), "270? %f", msg->ranges[270]);
    RCLCPP_INFO(this->get_logger(), "540? %f", msg->ranges[540]);
    RCLCPP_INFO(this->get_logger(), "810? %f", msg->ranges[810]);
    RCLCPP_INFO(this->get_logger(), "1080? %f", msg->ranges[1080]);
  }

  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = speed_linear_x;
    // message.angular.z = speed_angular_z;
    vel_pub_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Moving");
  }

private:
  // attributes
  // call back groups
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;

  // sub and pubs
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  double speed_linear_x, speed_angular_z;

  // arg params
  double arg_obstacle;
  int arg_degrees;
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