#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <iomanip>
#include <ostream>

using namespace std::chrono_literals;
using std::placeholders::_1;
using Odometry = nav_msgs::msg::Odometry;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Twist = geometry_msgs::msg::Twist;

std::array<Eigen::Vector3f, 8> relative_waypoints = {{{0.0, 1.0, -1.0},
                                                      {0.0, 1.0, 1.0},
                                                      {0.0, 1.0, 1.0},
                                                      {-1.5708, 1.0, -1.0},
                                                      {-1.5708, -1.0, -1.0},
                                                      {0.0, -1.0, 1.0},
                                                      {0.0, -1.0, 1.0},
                                                      {0.0, -1.0, -1.0}}};

constexpr float l = 0.085;
constexpr float w = 0.134845;
constexpr float r = 0.05;

Eigen::Matrix<float, 4, 3> H{{(-l - w) / r, 1.0f / r, -1.0f / r},
                             {(l + w) / r, 1.0f / r, 1.0f / r},
                             {(l + w) / r, 1.0f / r, -1.0f / r},
                             {(-l - w) / r, 1.0f / r, 1.0f / r}};

constexpr float kp_xy = 2.0;
constexpr float kp_phi = 2.0;
constexpr float max_linear = 2.0;
constexpr float max_angular = 2.0;
constexpr float min_phi_error = 0.05;
constexpr float min_xy_error = 0.01;

Eigen::Vector3f K_p{kp_phi, kp_xy, kp_xy};

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory") {
    odom_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    odom_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*odom_tf_buffer_);
    wheel_speed_publisher_ = this->create_publisher<Float32MultiArray>("/wheel_speed", 10);
    cmd_vel_publisher_ = this->create_publisher<Twist>("/cmd_vel", 10);

    rclcpp::sleep_for(500ms);

    odom_timer_ = this->create_wall_timer(20ms, std::bind(&EightTrajectory::odom_tf_lookup, this));
    trajectory_timer_ = this->create_wall_timer(20ms, std::bind(&EightTrajectory::execute_trajectory, this));
    logging_timer_ = this->create_wall_timer(500ms, std::bind(&EightTrajectory::print_logging, this));

    wheel_speed_msg_ = Float32MultiArray();
    wheel_speed_msg_.data.resize(4);
    waypoint_idx_ = 0;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> odom_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> odom_tf_listener_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr wheel_speed_publisher_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr trajectory_timer_;
  rclcpp::TimerBase::SharedPtr logging_timer_;
  Float32MultiArray wheel_speed_msg_;
  Eigen::Vector3f goal_pose_s_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f odom_pose_s_, error_s_, error_b_, twist_s_, twist_b_;
  size_t waypoint_idx_;

  void odom_tf_lookup() {
    try {
      geometry_msgs::msg::TransformStamped tf =
          odom_tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
      auto q = tf.transform.rotation;
      odom_pose_s_(0) = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      odom_pose_s_(1) = tf.transform.translation.x;
      odom_pose_s_(2) = tf.transform.translation.y;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup odom --> base_link failed: %s", ex.what());
    }
  }

  bool goal_reached() {
    return (std::fabs(error_s_(0)) < min_phi_error && std::hypot(error_s_(1), error_s_(2)) < min_xy_error);
  }

  void wrap_angle(Eigen::Vector3f vec) {
    // Constraint angle phi to [-pi, +pi]
    if (vec(0) > M_PI)
      vec(0) -= 2 * M_PI;
    else if (vec(0) < -M_PI)
      vec(0) += 2 * M_PI;
  }

  void execute_trajectory() {
    if (goal_reached()) {
      if (waypoint_idx_ < relative_waypoints.size()) {
        // Add relative waypoint to goal pose in the static frame
        goal_pose_s_ += relative_waypoints[waypoint_idx_];
        wrap_angle(goal_pose_s_);
      } else {
        // Finished trajectory --> publish zero twist
        std::fill(wheel_speed_msg_.data.begin(), wheel_speed_msg_.data.end(), 0.0f);
        wheel_speed_publisher_->publish(wheel_speed_msg_);

        odom_timer_->cancel();
        trajectory_timer_->cancel();
        logging_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "========================================================");
        RCLCPP_INFO(this->get_logger(), "Finished trajectory!");

        return;
      }

      waypoint_idx_++;

      RCLCPP_INFO(this->get_logger(), "========================================================");
      RCLCPP_INFO(this->get_logger(), "Going to waypoint %zu", waypoint_idx_);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Odom pose (s):  " << std::fixed << std::setprecision(2) << odom_pose_s_.transpose());
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Goal pose (s):  " << std::fixed << std::setprecision(2) << goal_pose_s_.transpose());
    }

    // Get static frame error
    error_s_ = goal_pose_s_ - odom_pose_s_;
    wrap_angle(error_s_);

    // Apply SE(3) rotation to get body frame errors
    Eigen::Matrix3f R_bs = Eigen::AngleAxisf(-odom_pose_s_(0), Eigen::Vector3f::UnitX()).toRotationMatrix();
    error_b_ = R_bs * error_s_;

    // Apply proportional scaling to get body twist
    twist_b_ = K_p.cwiseProduct(error_b_);

    // Convert body twist to wheel speeds
    Eigen::Vector4f u = H * twist_b_;

    // std::copy(u.data(), u.data() + u.size(), wheel_speed_msg_.data.begin());
    Eigen::Map<Eigen::Vector4f>(wheel_speed_msg_.data.data()) = u;
    wheel_speed_publisher_->publish(wheel_speed_msg_);
  }

  void print_logging() {
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
    RCLCPP_INFO_STREAM(this->get_logger(), "Error (s):  " << std::fixed << std::setprecision(2) << error_s_.transpose()
                                                          << "\t(b):  " << std::fixed << std::setprecision(2)
                                                          << error_b_.transpose());
    RCLCPP_INFO_STREAM(this->get_logger(), "Twist (s):  " << std::fixed << std::setprecision(2) << twist_s_.transpose()
                                                          << "\t(b):  " << std::fixed << std::setprecision(2)
                                                          << twist_b_.transpose());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EightTrajectory>());
  rclcpp::shutdown();
  return 0;
}