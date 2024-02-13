#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <attach_shelf/srv/go_to_loading.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>

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
    // laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "scan", qos_profile_subscriber,
    //     std::bind(&ApproachShelfService::scan_callback, this, _1), option1);

    // transform Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // transform listener from /odom to /laser
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // init cmd_vel publisher
    vel_pub_ =
        // this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel",
        // 10);
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // elevator pub
    lift_up_pub_ =
        this->create_publisher<std_msgs::msg::String>("elevator_up", 1);
    lift_down_pub_ =
        this->create_publisher<std_msgs::msg::String>("elevator_down", 1);

    // send elevator down, to ensure starting with elevator down
    this->lift_down();

    // init frames needed COULD BE PARAMETERS
    to_frame = "robot_cart_laser";
    from_frame = "robot_base_link";

    // server
    srv_ = create_service<GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachShelfService::GoToLoading_callback, this, _1, _2),
        rmw_qos_profile_services_default, callback_group_3);
  }

private:
  void
  GoToLoading_callback(const std::shared_ptr<GoToLoading::Request> request,
                       const std::shared_ptr<GoToLoading::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Final approach?: %s",
                request->attach_to_shelf ? "True" : "False");

    // set bool got request
    got_req = true;

    bool to_shelf = false;
    // check if lift is needed
    if (request->attach_to_shelf) {
      // TEST: testing to make sure that elevator works before approaching;
      RCLCPP_INFO(this->get_logger(),
                  "Testing elevator to make sure everything works before "
                  "approaching shelf.");
      this->lift_up();
      this->lift_down();

      // approach shelff
      to_shelf = approach_shelf();

      cout << " to shelf ? " << to_shelf << endl;

      // if going under shelf was successful
      if (to_shelf) {
        // lift cart
        lift_up();
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Within service handler, to shelf success? %s",
                to_shelf ? "True" : "False");

    response->complete = to_shelf;

    // // turn off tf broadcast
    // got_req = false;
  }

  void lift_up() {
    std_msgs::msg::String msg;
    msg.data = "";
    this->lift_up_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Lifting up...");
    this->moveForX(8, 0, 0);
  }

  void lift_down() {
    std_msgs::msg::String msg;
    msg.data = "";
    this->lift_down_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Lifting down...");
    this->moveForX(8, 0, 0);
  }

  bool approach_shelf(float cx = 0) {
    float THRESH_HOLD = 0.38;
    geometry_msgs::msg::TransformStamped t;

    // while (rclcpp::ok() && got_both_legs) {
    while (rclcpp::ok()) {

      // Look up transformation
      try {
        t = tf_buffer_->lookupTransform(from_frame, to_frame,
                                        tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                     to_frame.c_str(), from_frame.c_str(), ex.what());
        return false;
      }

      // calculate distance error
      float error_dist = sqrt(pow(t.transform.translation.x + cx, 2) +
                              pow(t.transform.translation.y, 2) +
                              pow(t.transform.translation.z, 2));

      RCLCPP_INFO(this->get_logger(), "error distance is %f", error_dist);

      // init kp
      float kp_distance = 0.15;
      float kp_yaw = 3;

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

      cout << "error_dist" << error_dist << endl;
      //   cout << "raw_linear_x " << raw_linear_x << endl;
      //   cout << "raw_angular_z" << raw_angular_z << endl;

      // close enought to target frame??
      if (abs(error_yaw) < 0.1 && error_dist < THRESH_HOLD) {

        // Tick: Record the start time
        auto start_time = std::chrono::high_resolution_clock::now();

        moveForX(4, 0.125, 0);

        // Tock: Record the end time
        auto end_time = std::chrono::high_resolution_clock::now();

        // Calculate the elapsed time
        auto elapsed_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                  start_time);

        // Print the elapsed time
        std::cout << "Elapsed time: " << elapsed_time.count() << " milliseconds"
                  << std::endl;

        // stop
        moveForX(1, 0, 0);
        return true;
      } else {
        // move
        move(raw_linear_x, raw_angular_z);
      }
      // sleep
      RCLCPP_INFO(this->get_logger(), "Sleeping in shelf approach loop");

      rclcpp::sleep_for(50ms);
      ;
    }
    // this should never be reached
    return false;
  }

  void moveForX(int x_sec, float linear_x, float angular_z) {
    // Set the loop rate to publish every 500 milliseconds
    rclcpp::WallRate loop_rate(10); // 10 Hz (publish every 100 milliseconds)

    // Publish Twist messages in a loop for 10 seconds
    auto start_time = this->now();
    while ((this->now() - start_time).seconds() < x_sec) { // Run for 10 seconds
      move(linear_x, angular_z);
      loop_rate.sleep(); // Sleep to achieve the specified loop rate
    }
  }

  void move(float linear_x, float angular_z, float MAX_LINEAR_X = 0.5,
            float MAX_ANGULAR_Z = 1) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = clamp(linear_x, -MAX_LINEAR_X, MAX_LINEAR_X);
    msg.angular.z = clamp(angular_z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z);
    vel_pub_->publish(msg);
  }

  vector<std::pair<int, int>>
  getConsecutiveRanges(const vector<float> &values,
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

  vector<int> getMidRangeIndex(const vector<std::pair<int, int>> &ranges) {

    vector<int> mid_range_indices;

    for (const auto &range : ranges) {
      int mid_range = (range.first + range.second) / 2;
      mid_range_indices.emplace_back(mid_range);
    }
    return mid_range_indices;
  }

  vector<pair<float, float>> getXYFromIndex(const vector<int> indices,
                                            const vector<float> dist_values,
                                            float angle_start,
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

  void setVec2MidPt(const vector<pair<float, float>> xy_pairs) {
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

  void broadcastCartTF(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
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

    // TODO TF BROADCASTER
    tf_broadcaster_->sendTransform(tf_togo);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Only Activatef request was recieved
    if (got_req) {
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
      //   if (got_both_legs) {
      //     broadcastCartTF(msg);
      //   }
    }
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::CallbackGroup::SharedPtr callback_group_3;

  rclcpp::Service<GoToLoading>::SharedPtr srv_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lift_up_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lift_down_pub_;

  // frames
  string to_frame;
  string from_frame;

  // vec2 mid_pt;
  std::pair<float, float> vec2mid_pt;

  // flags
  bool got_req;
  bool got_both_legs;
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