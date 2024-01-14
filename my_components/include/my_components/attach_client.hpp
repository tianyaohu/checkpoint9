#ifndef COMPOSITION__ATTACH_CLIENT_COMPONENT_HPP_
#define COMPOSITION__ATTACH_CLIENT_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <attach_shelf/srv/go_to_loading.hpp>

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

protected:
  void on_timer();

private:
  bool request_sent = false;

  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  void response_callback(
      rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future);
};

} // namespace my_components

#endif // COMPOSITION__ATTACH_CLIENT_COMPONENT_HPP_