#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <attach_shelf/srv/go_to_loading.hpp>
#include <numeric>
// #include "sensor_msgs/msg/laser_scan.hpp"

#include <memory>

using GoToLoading = attach_shelf::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std;

class ApproachShelfService : public rclcpp::Node {
public:
  ApproachShelfService() : Node("approach_shelf_service_server") {
    srv_ = create_service<GoToLoading>(
        "approach_shelf",
        std::bind(&ApproachShelfService::GoToLoading_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GoToLoading>::SharedPtr srv_;

  void
  GoToLoading_callback(const std::shared_ptr<GoToLoading::Request> request,
                       const std::shared_ptr<GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(), "attach2shelf request: %s",
                request->attach_to_shelf ? "True" : "False");

    response->complete = false;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachShelfService>());
  rclcpp::shutdown();
  return 0;
}