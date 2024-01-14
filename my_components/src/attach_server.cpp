#include "my_components/attach_server.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"

#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <utility>
#include <vector>

std::mutex vec_mu;

using GoToLoading = attach_shelf::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std;

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("approach_shelf_service_server", options) {

  // QOS profile
  rclcpp::QoS qos_profile_subscriber(1);
  qos_profile_subscriber.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // init laser subscription with QOS
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos_profile_subscriber,
      std::bind(&AttachServer::scan_callback, this, _1));

  // transform Broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // transform listener from /odom to /laser
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // init frames names for listener
  to_frame = "cart_frame";
  from_frame = "robot_base_link";

  // init cmd_vel publisher
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  // elevator pub
  lift_up_pub_ =
      this->create_publisher<std_msgs::msg::String>("/elevator_up", 1);
  lift_down_pub_ =
      this->create_publisher<std_msgs::msg::String>("/elevator_down", 1);

  // server
  srv_ = create_service<GoToLoading>(
      "approach_shelf",
      std::bind(&AttachServer::GoToLoading_callback, this, _1, _2));

  timer_ = create_wall_timer(100ms, std::bind(&AttachServer::on_timer, this));
}

void AttachServer::lift_up() {
  std_msgs::msg::String msg;
  lift_up_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Lifting?:");
}

void AttachServer::lift_down() {
  std_msgs::msg::String msg;
  lift_down_pub_->publish(msg);
}

void AttachServer::move(float linear_x, float angular_z,
                        float MAX_LINEAR_X = 0.5, float MAX_ANGULAR_Z = 1) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = clamp(linear_x, -MAX_LINEAR_X, MAX_LINEAR_X);
  msg.angular.z = clamp(angular_z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z);
  vel_pub_->publish(msg);
}

void AttachServer::moveForX(int x_sec, float linear_x, float angular_z) {
  // Set the loop rate to publish every 500 milliseconds
  rclcpp::WallRate loop_rate(10); // 10 Hz (publish every 100 milliseconds)

  // Publish Twist messages in a loop for 10 seconds
  auto start_time = this->now();
  while ((this->now() - start_time).seconds() < x_sec) { // Run for 10 seconds
    move(linear_x, angular_z);
    loop_rate.sleep(); // Sleep to achieve the specified loop rate
  }
}

void AttachServer::GoToLoading_callback(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Final approach?: %s",
              request->attach_to_shelf ? "True" : "False");

  // set bool got request
  got_req = true;

  bool to_shelf = false;
  // check if doing final approach
  if (request->attach_to_shelf) {
    // force sleeping just to wait on broadcast for 2 sec
    moveForX(2, 0, 0);

    // approach shelff
    go2cart = true;
  }

  RCLCPP_INFO(this->get_logger(),
              "Within service handler, to shelf success? %s",
              to_shelf ? "True" : "False");

  response->complete = reached_cart;
}

void AttachServer::on_timer() {
  if (go2cart) {

    float THRESH_HOLD = 0.24;
    geometry_msgs::msg::TransformStamped t;

    // Look up transformation
    try {
      t = tf_buffer_->lookupTransform(from_frame, to_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                   to_frame.c_str(), from_frame.c_str(), ex.what());
      return;
    }

    // calculate distance error
    float error_dist = sqrt(pow(t.transform.translation.x, 2) +
                            pow(t.transform.translation.y, 2) +
                            pow(t.transform.translation.z, 2));

    RCLCPP_INFO(this->get_logger(), "error distance is %f", error_dist);

    // init kp
    float kp_distance = 1;
    float kp_yaw = 1.5;

    // calculate true delta
    float error_yaw;
    if (error_dist < THRESH_HOLD * 2) {

      // init quaternion
      tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y,
                        t.transform.rotation.z, t.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, error_yaw;
      m.getRPY(roll, pitch, error_yaw);

      RCLCPP_INFO(this->get_logger(), "close enoough, error yaw is %f",
                  error_yaw);

    } else {
      error_yaw = atan2(t.transform.translation.y, t.transform.translation.x);

      RCLCPP_INFO(this->get_logger(), "far , error yaw is %f", error_yaw);
    }

    // raw speed
    float raw_linear_x = kp_distance * error_dist;
    float raw_angular_z = kp_yaw * error_yaw;

    // close enought to target frame??
    if (abs(error_yaw) < THRESH_HOLD && error_dist < THRESH_HOLD) {

      // moving the extra 30 cm to underneath the cart
      if (extra_time > 0) {
        move(0.2, 0);

      } else {
        // stop
        move(0, 0);

        reached_cart = true;
        RCLCPP_INFO(this->get_logger(), "Reached Cart");

        if (extra_time == 0) {
          // lift cart
          lift_up();
        }
      }
      extra_time--;

    } else {
      // move
      move(raw_linear_x, raw_angular_z);
    }
  }
}

vector<std::pair<int, int>>
AttachServer::getConsecutiveRanges(const vector<float> &values,
                                   const float THRESH_HOLD = 0.001) {
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
AttachServer::getMidRangeIndex(const vector<std::pair<int, int>> &ranges) {

  vector<int> mid_range_indices;

  for (const auto &range : ranges) {
    int mid_range = (range.first + range.second) / 2;
    mid_range_indices.emplace_back(mid_range);
  }
  return mid_range_indices;
}

vector<pair<float, float>>
AttachServer::getXYFromIndex(const vector<int> indices,
                             const vector<float> dist_values, float angle_start,
                             float angle_increment) {
  vector<pair<float, float>> vec_xy;
  for (const int &index : indices) {

    // calc angle and distance with index
    float rad = angle_start + angle_increment * index;
    float dist = dist_values[index];

    // calculate x and y
    float x = cos(rad) * dist;
    float y = sin(rad) * dist;

    // create pair and store
    vec_xy.emplace_back(make_pair(x, y));
  }
  return vec_xy;
}

void AttachServer::setVec2MidPt(const vector<pair<float, float>> xy_pairs) {
  // MUTEX LOCK HERE
  vec_mu.lock();

  // combine the pair
  vec2mid_pt = std::make_pair(xy_pairs[0].first + xy_pairs[1].first,
                              xy_pairs[0].second + xy_pairs[1].second);
  // halfing
  vec2mid_pt.first /= 2;
  vec2mid_pt.second /= 2;

  // MUTEX UNLOCK HERE
  vec_mu.unlock();
}

void AttachServer::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (got_req && !reached_cart) {
    // find the ranges with consecutive indices with values greater
    vector<pair<int, int>> high_intense_ranges =
        getConsecutiveRanges(msg->intensities);

    // Check cart leg count
    if (high_intense_ranges.size() == 2) {
      // set flag for tf read
      got_both_legs = true;

      // find the indices with min value
      vector<int> min_indices = getMidRangeIndex(high_intense_ranges);

      // convert min indices into xy vectors
      vector<pair<float, float>> vec_xy = getXYFromIndex(
          min_indices, msg->ranges, msg->angle_min, msg->angle_increment);

      // get vector pointing to the mid_pt between cart legs
      setVec2MidPt(vec_xy);

    } else {
      got_both_legs = false;
      // check if there are
      RCLCPP_ERROR(this->get_logger(),
                   "MUST have exactly two legs to do broadcast");
    }

    // BroadCast TF if both legs of cart are detected
    if (got_both_legs) {
      broadcastCartTF(msg);
    }
  }
}

void AttachServer::broadcastCartTF(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  // init transformstamped
  geometry_msgs::msg::TransformStamped tf_togo;
  // corresponding tf variables
  tf_togo.header.stamp = scan_msg->header.stamp;
  tf_togo.header.frame_id = "robot_front_laser_base_link";
  tf_togo.child_frame_id = "cart_frame";

  // set translation coordinates
  vec_mu.lock();
  tf_togo.transform.translation.x = vec2mid_pt.first;
  tf_togo.transform.translation.y = vec2mid_pt.second;
  tf_togo.transform.translation.z = 0; // asssuming perfectly flat ground
  vec_mu.unlock();

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  tf_togo.transform.rotation.x = q.x();
  tf_togo.transform.rotation.y = q.y();
  tf_togo.transform.rotation.z = q.z();
  tf_togo.transform.rotation.w = q.w();

  // TF BROADCASTER
  tf_broadcaster_->sendTransform(tf_togo);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)