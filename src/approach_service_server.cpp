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



  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {


  // find the ranges with consecutive indices with values greater

  std::cout << "------------------------------------" << endl;

  cout << "length of intensity scan" << msg->intensities.size() << endl;

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
}
;

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