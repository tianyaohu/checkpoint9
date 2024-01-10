#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// custom srv
#include <attach_shelf/srv/go_to_loading.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace std;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node") {
    // parameter descriptions
    auto obstacle_desc = rcl_interfaces::msg::ParameterDescriptor{};
    auto degree_desc = rcl_interfaces::msg::ParameterDescriptor{};
    auto final_approach_desc = rcl_interfaces::msg::ParameterDescriptor{};

    // param description details
    obstacle_desc.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";
    degree_desc.description =
        "Number of degrees for the rotation of the robot after stopping";
    final_approach_desc.description =
        "If True, the robot will move underneath the shelf and it will lift "
        "it. If False, the robot will only publish the cart_frame transform, "
        "but will not do the final approach";

    // declearing parameters
    this->declare_parameter<std::double_t>("obstacle", 2.0, obstacle_desc);
    this->declare_parameter<int>("degrees", 360, degree_desc);
    this->declare_parameter<bool>("final_approach", true, final_approach_desc);

    // set and print to see if parameter Setting workedf
    this->get_parameter("obstacle", arg_obstacle);
    RCLCPP_INFO(this->get_logger(), "Obstacle parameter is: %f", arg_obstacle);

    this->get_parameter("degrees", arg_degrees);
    RCLCPP_INFO(this->get_logger(), "Degrees parameter is: %d", arg_degrees);

    this->get_parameter("final_approach", arg_final_approach);
    RCLCPP_INFO(this->get_logger(), "Degrees parameter is: %s",
                arg_final_approach ? "True" : "False");

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

    // init approach service client
    client_ =
        this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

    // set initial speed
    speed_linear_x = 0.2;
    speed_angular_z = 0.0;

    MAX_ANGULAR_SPEED = 0.5;
  }

private:
  // call back groups
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::CallbackGroup::SharedPtr callback_group_3;

  // sub and pubs
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // client
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
  bool attach_load_service_done_ = false;

  // speed
  double MAX_ANGULAR_SPEED;
  float speed_linear_x, speed_angular_z;

  // arg params
  double arg_obstacle;
  int arg_degrees;
  bool arg_final_approach;

  // pre approach params: wall flag;
  bool wall_reached = false;
  bool finished_rot = false;

  float cur_yaw;
  float target_yaw;

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
      // init target yaw
      init_target_yaw();
      wall_reached = true;
    }
  }

  void init_target_yaw() {
    // get current yaw
    target_yaw = cur_yaw + (arg_degrees % 360) * M_PI / 180;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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

  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = speed_linear_x;
    message.angular.z = speed_angular_z;
    vel_pub_->publish(message);
  }

  void rotate() {
    // calc turn delta
    double turn_delta = cur_yaw - target_yaw;
    // normalize turn delta
    if (abs(turn_delta) > M_PI) {
      turn_delta =
          turn_delta > 0 ? -2 * M_PI + turn_delta : 2 * M_PI + turn_delta;
    }
    // set angular speed
    speed_angular_z = min(MAX_ANGULAR_SPEED, turn_delta / -2);

    RCLCPP_INFO(this->get_logger(), "turn_delta? %f", turn_delta);
    RCLCPP_INFO(this->get_logger(), "cur_yaw? %f", cur_yaw);
    RCLCPP_INFO(this->get_logger(), "finished_rot? %s",
                finished_rot ? "True" : "False");

    if (turn_delta < 0.05) {
      speed_angular_z = 0;
      finished_rot = true;
      RCLCPP_INFO(this->get_logger(), "Final Yaw upon finishing? %f", cur_yaw);
      // call approach service
      call_approach_service();
    }
  }

  void call_approach_service() {
    // block to wait for service to become available
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Client interrupted while waiting for service. "
                     "Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }
    // service to became available

    // init request
    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = true; // temp need to add one more argument
    // set service status flag to false;
    attach_load_service_done_ = false;
    // send request
    auto result_future = client_->async_send_request(
        request, std::bind(&PreApproach::response_callback, this,
                           std::placeholders::_1));
  }

  void response_callback(
      rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
    auto status = future.wait_for(1s);
    // if result ready
    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Result: success %s",
                  future.get()->complete ? "True" : "False");
      // raise flag service is done
      attach_load_service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
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