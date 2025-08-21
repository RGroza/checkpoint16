#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <array>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using Odometry = nav_msgs::msg::Odometry;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;

// constexpr std::array<std::array<float, 3>, 8> trajectory_waypoints = {{{0.0, 1.0, -1.0},
//                                                                        {0.0, 1.0, 1.0},
//                                                                        {0.0, 1.0, 1.0},
//                                                                        {-1.5708, 1.0, -1.0},
//                                                                        {-1.5708, -1.0, -1.0},
//                                                                        {0.0, -1.0, 1.0},
//                                                                        {0.0, -1.0, 1.0},
//                                                                        {0.0, -1.0, -1.0}}};

constexpr std::array<std::array<float, 3>, 1> trajectory_waypoints = {{{1.5708, 1.0, 0.0}}};

constexpr float waypoint_time_sec = 1.0;

constexpr float l = 0.085;
constexpr float w = 0.134845;
constexpr float r = 0.05;

constexpr float H[4][3] = {{(-l - w) / r, 1.0f / r, -1.0f / r},
                           {(l + w) / r, 1.0f / r, 1.0f / r},
                           {(l + w) / r, 1.0f / r, -1.0f / r},
                           {(-l - w) / r, 1.0f / r, 1.0f / r}};

struct OdomPose {
  float x;
  float y;
  float phi;
};

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory"), waypoint_duration_(rclcpp::Duration::from_seconds(waypoint_time_sec)) {
    odom_subscriber_ = this->create_subscription<Odometry>("rosbot_xl_base_controller/odom", 10,
                                                           std::bind(&EightTrajectory::odom_callback, this, _1));

    wheel_speed_publisher_ = this->create_publisher<Float32MultiArray>("/wheel_speed", 10);

    timer_ = this->create_wall_timer(20ms, std::bind(&EightTrajectory::execute_trajectory, this));

    wheel_speed_msg_ = Float32MultiArray();
    wheel_speed_msg_.data.resize(4);

    first_execute_ = true;
    waypoint_idx_ = 0;
  }

private:
  rclcpp::Subscription<Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr wheel_speed_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Duration waypoint_duration_;
  rclcpp::Time start_time_;
  Float32MultiArray wheel_speed_msg_;
  bool first_execute_;
  size_t waypoint_idx_;
  OdomPose odom_pose_;

  float phi_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void odom_callback(const Odometry::SharedPtr msg) {
    odom_pose_.x = msg->pose.pose.position.x;
    odom_pose_.y = msg->pose.pose.position.y;
    odom_pose_.phi = phi_from_quaternion(msg->pose.pose.orientation);
  }

  std::array<float, 4> velocity_to_wheel_speeds(float dphi, float dx, float dy) {
    const float c = std::cos(odom_pose_.phi);
    const float s = std::sin(odom_pose_.phi);

    float v[3];
    v[0] = dphi;
    v[1] = c * dx + s * dy;
    v[2] = -s * dx + c * dy;

    std::array<float, 4> u{};
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        u[i] += H[i][j] * v[j];
      }
    }
    return u;
  }

  void execute_trajectory() {
    if (first_execute_) {
      start_time_ = this->get_clock()->now();
      first_execute_ = false;
      RCLCPP_INFO(this->get_logger(), "odom: (%f, %f, %f)", odom_pose_.x, odom_pose_.y, odom_pose_.phi);
    }

    if (waypoint_idx_ < trajectory_waypoints.size()) {
      auto w = trajectory_waypoints[waypoint_idx_];

      const float wz = w[0];
      const float vx = w[1];
      const float vy = w[2];

      auto u = velocity_to_wheel_speeds(wz, vx, vy);
      std::copy(u.begin(), u.end(), wheel_speed_msg_.data.begin());
      wheel_speed_publisher_->publish(wheel_speed_msg_);

      if (this->get_clock()->now() - start_time_ > waypoint_duration_) {
        RCLCPP_INFO(this->get_logger(), "Finished waypoint %zu", waypoint_idx_ + 1);
        RCLCPP_INFO(this->get_logger(), "odom: (%f, %f, %f)", odom_pose_.x, odom_pose_.y, odom_pose_.phi);
        waypoint_idx_++;
        start_time_ = this->get_clock()->now();
      }
    } else {
      std::fill(wheel_speed_msg_.data.begin(), wheel_speed_msg_.data.end(), 0.0f);
      wheel_speed_publisher_->publish(wheel_speed_msg_);
      timer_->cancel();
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EightTrajectory>());
  rclcpp::shutdown();
  return 0;
}