#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <attach_shelf/srv/go_to_loading.hpp>

#include <cmath>
#include <cstddef>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

using GoToLoading = attach_shelf::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std;

class ApproachShelfService : public rclcpp::Node {
public:
  ApproachShelfService() : Node("approach_shelf_service_server") {

    // init callbackgroups
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    RCLCPP_INFO(this->get_logger(), "After Callbaack group");

    // init subscription options
    rclcpp::SubscriptionOptions option1;
    option1.callback_group = callback_group_1;

    // QOS profile
    rclcpp::QoS qos_profile_subscriber(1);
    qos_profile_subscriber.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // init laser subscription with QOS
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos_profile_subscriber,
        std::bind(&ApproachShelfService::scan_callback, this, _1), option1);

    // transform Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // transform listener from /odom to /laser
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // timer for triggering listener
    timer_ = this->create_wall_timer(
        10ms, std::bind(&ApproachShelfService::timer_callback, this),
        callback_group_2);

    // server
    srv_ = create_service<GoToLoading>(
        "approach_shelf",
        std::bind(&ApproachShelfService::GoToLoading_callback, this, _1, _2),
        rmw_qos_profile_services_default, callback_group_3);
  }

private:
  void timer_callback() {
    // init frames needed
    string target_frame = "robot_front_laser_link";
    string robot_odom = "robot_odom";
    // Look up transformation
    try {
      t = std::make_shared<geometry_msgs::msg::TransformStamped>(
          tf_buffer_->lookupTransform(target_frame, robot_odom,
                                      tf2::TimePointZero));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  target_frame.c_str(), robot_odom.c_str(), ex.what());
      return;
    }

    // Do something with the transform (e.g., print it)
    RCLCPP_INFO(get_logger(), "Received transform: [%f, %f, %f]",
                t->transform.translation.x, t->transform.translation.y,
                t->transform.translation.z);
  }

  vector<std::pair<int, int>>
  getConsecutiveRanges(const vector<float> &values) {
    float THRESH_HOLD = 0.001;
    vector<std::pair<int, int>> ranges;

    size_t start = 0;
    size_t end = 0;

    for (size_t i = 0; i < values.size(); ++i) {
      if (values[i] > THRESH_HOLD) {
        // Current value is greater than 0, extend the range
        end = i;
      } else {
        // Current value is not greater than 0, add the range if it exists
        if (start != end) {
          ranges.emplace_back(start, end - 1);
        }
        // Reset the range
        start = end = i + 1;
      }
    }

    // Add the last range if it exists
    if (start != end) {
      ranges.emplace_back(start, end - 1);
    }

    return ranges;
  }

  vector<int>
  findMinIndicesInRanges(const vector<float> &values,
                         const vector<std::pair<int, int>> &ranges) {

    vector<int> minIndices;

    for (const auto &range : ranges) {
      // Find the minimum value and its index within the range
      auto iterator = min_element(values.begin() + range.first,
                                  values.begin() + range.second);
      // Calculate the index of the maximum element
      int index = distance(values.begin(), iterator);
      minIndices.emplace_back(index);
    }
    return minIndices;
  }

  vector<pair<float, float>> getXYFromIndex(const vector<int> indices,
                                            const vector<float> dist_values,
                                            int len_half_range) {
    vector<pair<float, float>> vec_xy;
    for (const auto &index : indices) {
      float rad = static_cast<float>(len_half_range - index) / len_half_range *
                  M_PI; // here assumes that indices lower than half range
                        // represent left (negative rads)
      float dist = dist_values[index];

      cout << "index is " << index << endl;
      cout << "rad is " << rad << endl;
      cout << "dist is " << dist << endl;

      // calc x and y
      float x = sin(rad + M_PI_2) * dist;
      float y = cos(rad + M_PI_2) * dist;

      cout << "x is " << x << endl;
      cout << "y is " << y << endl;

      // create pair and store
      vec_xy.emplace_back(make_pair(x, y));
    }
    return vec_xy;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // find the ranges with consecutive indices with values greater
    vector<pair<int, int>> high_intense_ranges =
        getConsecutiveRanges(msg->intensities);

    // find the indices with min value
    vector<int> min_indices =
        findMinIndicesInRanges(msg->ranges, high_intense_ranges);

    // convert min indices into xy vectors
    vector<pair<float, float>> vec_xy =
        getXYFromIndex(min_indices, msg->ranges, msg->ranges.size() / 2);

    // check if there is strictly vec_xy has two values
    if (vec_xy.size() == 2) {

      pair<float, float> vec2mid_pt =
          make_pair(vec_xy[0].first + vec_xy[1].first,
                    vec_xy[0].second + vec_xy[1].second);

      cout << "vec2mid_pt first " << vec2mid_pt.first << "; vec2mid_pt second "
           << vec2mid_pt.second << endl;
    }

    // init transformstamped
    // geometry_msgs::msg::TransformStamped t;
    // // corresponding tf variables
    // t.header.stamp = this->get_clock()->now();
    // t.header.frame_id = "robot_front_laser_base_link";
    // t.child_frame_id = "cart_frame";

    // coordinates from the message and set the z coordinate to 0
    // t.transform.translation.x = cur_pos.x;
    // t.transform.translation.y = cur_pos.y;
    // t.transform.translation.z = cur_z;

    // TODO TF BROADCASTER

    std::cout << "------------------------------------" << endl;
  }

  void
  GoToLoading_callback(const std::shared_ptr<GoToLoading::Request> request,
                       const std::shared_ptr<GoToLoading::Response> response) {

    RCLCPP_INFO(this->get_logger(), "attach2shelf request: %s",
                request->attach_to_shelf ? "True" : "False");

    response->complete = false;
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::CallbackGroup::SharedPtr callback_group_3;

  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // flags
  bool got_req;

  // transform looked up
  std::shared_ptr<geometry_msgs::msg::TransformStamped> t;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // init node
  std::shared_ptr<ApproachShelfService> node =
      std::make_shared<ApproachShelfService>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}