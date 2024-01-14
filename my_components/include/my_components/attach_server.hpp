#ifndef COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_
#define COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <attach_shelf/srv/go_to_loading.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

using GoToLoading = attach_shelf::srv::GoToLoading;
using namespace std;

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

private:
  void lift_up();
  void lift_down();
  void on_timer();
  void move(float linear_x, float angular_z, float MAX_LINEAR_X,
            float MAX_ANGULAR_Z);
  void moveForX(int x_sec, float linear_x, float angular_z);

  void
  GoToLoading_callback(const std::shared_ptr<GoToLoading::Request> request,
                       const std::shared_ptr<GoToLoading::Response> response);

  vector<std::pair<int, int>> getConsecutiveRanges(const vector<float> &values,
                                                   const float THRESH_HOLD);

  vector<int> getMidRangeIndex(const vector<std::pair<int, int>> &ranges);
  vector<pair<float, float>> getXYFromIndex(const vector<int> indices,
                                            const vector<float> dist_values,
                                            float angle_start,
                                            float angle_increment);
  void setVec2MidPt(const vector<pair<float, float>> xy_pairs);

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void broadcastCartTF(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lift_up_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lift_down_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // frames
  string to_frame;
  string from_frame;

  // vec2 mid_pt;
  std::pair<float, float> vec2mid_pt;

  // flags
  bool got_req;
  bool got_both_legs;

  bool go2cart = false;
  bool reached_cart = false;

  int extra_time = 20;
};

} // namespace my_components

#endif // COMPOSITION__ATTACH_SERVER_COMPONENT_HPP_