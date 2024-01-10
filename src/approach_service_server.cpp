#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <attach_shelf/srv/go_to_loading.hpp>
#include <cstddef>
#include <numeric>

#include <memory>
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
        "/scan", qos_profile_subscriber,
        std::bind(&ApproachShelfService::scan_callback, this, _1), option1);

    // transform Broadcaster TODO

    // srv_ = create_service<GoToLoading>(
    //     "approach_shelf",
    //     std::bind(&ApproachShelfService::GoToLoading_callback, this, _1, _2),
    //     rmw_qos_profile_services_default, callback_group_2);
  }

private:
  void
  GoToLoading_callback(const std::shared_ptr<GoToLoading::Request> request,
                       const std::shared_ptr<GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(), "attach2shelf request: %s",
                request->attach_to_shelf ? "True" : "False");

    response->complete = false;
  }

  std::vector<std::pair<int, int>>
  getConsecutiveRanges(const std::vector<float> &values) {
    float THRESH_HOLD = 0.001;
    std::vector<std::pair<int, int>> ranges;

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

  std::vector<int>
  findMinIndicesInRanges(const std::vector<float> &values,
                         const std::vector<std::pair<int, int>> &ranges) {

    std::vector<int> minIndices;

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

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // find the ranges with consecutive indices with values greater
    vector<pair<int, int>> high_intense_ranges =
        getConsecutiveRanges(msg->intensities);

    // find the indicies with min value
    vector<int> min_indicies =
        findMinIndicesInRanges(msg->ranges, high_intense_ranges);

    // print to see min indicies
    for (const auto &i : min_indicies) {
      RCLCPP_INFO(this->get_logger(),
                  "min indicies : %d; corresponding value: %f", i,
                  msg->ranges[i]);
    }

    std::cout << "------------------------------------" << endl;

    // cout << "length of intensity scan" << msg->intensities.size() << endl;

    // print high intensity ranges
    // for (size_t i = 0; i < msg->intensities.size(); i++) {

    //   float temp = msg->intensities[i];
    //   if (temp > 0.001) {
    //     RCLCPP_INFO(this->get_logger(), "%ldth intensity reading: %f", i,
    //                 msg->intensities[i]);
    //   }
    // }

    // find the high intensity consecutive ranges
  }

  //   rclcpp::Service<GoToLoading>::SharedPtr srv_;

  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::CallbackGroup::SharedPtr callback_group_3;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
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