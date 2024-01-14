#include "my_components/attach_client.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("approach_shelf_client", options) {
  client_ =
      this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
  timer_ = create_wall_timer(100ms, std::bind(&AttachClient::on_timer, this));
}

void AttachClient::on_timer() {
  if (!request_sent) {
    // set request_sent to true
    request_sent = true;

    if (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
      return;
    }

    // init request
    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = true;

    // send request
    auto result_future = client_->async_send_request(
        request, std::bind(&AttachClient::response_callback, this,
                           std::placeholders::_1));
  }
}

void AttachClient::response_callback(
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
  auto status = future.wait_for(1s);
  // if result ready
  if (status == std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Result: success %s",
                future.get()->complete ? "True" : "False");

  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)