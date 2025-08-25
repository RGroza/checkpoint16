#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <array>
#include <cmath>
#include <iomanip>
#include <ostream>

using namespace std::chrono_literals;
using std::placeholders::_1;
using Odometry = nav_msgs::msg::Odometry;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Twist = geometry_msgs::msg::Twist;

constexpr std::array<std::array<float, 3>, 8> relative_waypoints = {{{0.0, 1.0, -1.0},
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

constexpr float H[4][3] = {{(-l - w) / r, 1.0f / r, -1.0f / r},
                           {(l + w) / r, 1.0f / r, 1.0f / r},
                           {(l + w) / r, 1.0f / r, -1.0f / r},
                           {(-l - w) / r, 1.0f / r, 1.0f / r}};

class OdomPose {
public:
  OdomPose() : phi_(0.0), x_(0.0), y_(0.0) {}

  OdomPose(float phi, float x, float y) : phi_(phi), x_(x), y_(y) {}

  OdomPose(std::array<float, 3> waypoint) : phi_(waypoint[0]), x_(waypoint[1]), y_(waypoint[2]) {}

  void set(float phi, float x, float y) {
    phi_ = phi;
    x_ = x;
    y_ = y;
  }

  OdomPose operator+(const OdomPose &p) const {
    return OdomPose(phi_ + p.phi_, x_ + p.x_, y_ + p.y_);
  }

  OdomPose &operator+=(const OdomPose &p) {
    phi_ += p.phi_;
    x_ += p.x_;
    y_ += p.y_;
    return *this;
  }

  void odom_callback(const Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    auto q = msg->pose.pose.orientation;
    phi_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  float error_x(const OdomPose &goal) const {
    const float ex = goal.x_ - x_;
    const float ey = goal.y_ - y_;
    const float c = std::cos(phi_);
    const float s = std::sin(phi_);
    return  c*ex + s*ey;
  }

  float error_y(const OdomPose &goal) const {
    const float ex = goal.x_ - x_;
    const float ey = goal.y_ - y_;
    const float c = std::cos(phi_);
    const float s = std::sin(phi_);
    return -s*ex + c*ey;
  }

  float error_phi(const OdomPose &goal_pose) const {
    float d = goal_pose.phi_ - phi_;
    if (d > M_PI)
      d -= 2 * M_PI;
    else if (d < -M_PI)
      d += 2 * M_PI;
    return d;
  }

  bool goal_reached(const OdomPose &goal_pose) const {
    return (std::fabs(error_phi(goal_pose)) < min_phi_error_ &&
            std::hypot(error_x(goal_pose), error_y(goal_pose)) < min_xy_error_);
  }

  std::string to_string() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "(" << phi_ << ", " << x_ << ", " << y_ << ")";
    return oss.str();
  }

  std::string error_to_string(const OdomPose &goal_pose) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "{phi: " << error_phi(goal_pose) << ", x: " << error_x(goal_pose) << ", y: " << error_y(goal_pose) << "}";
    return oss.str();
  }

  std::string goal_error_to_string(const OdomPose &goal_pose) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "{phi: " << std::fabs(error_phi(goal_pose)) << ", xy: " << std::hypot(error_x(goal_pose), error_y(goal_pose))
        << "}";
    return oss.str();
  }

private:
  float phi_, x_, y_;
  static constexpr float min_phi_error_ = 0.05f;
  static constexpr float min_xy_error_ = 0.01f;
};

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory") {
    // odom_subscriber_ = this->create_subscription<Odometry>("rosbot_xl_base_controller/odom", 10,
    //                                                        std::bind(&OdomPose::odom_callback, &odom_pose_, _1));
    odom_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    odom_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*odom_tf_buffer_);
    wheel_speed_publisher_ = this->create_publisher<Float32MultiArray>("/wheel_speed", 10);
    cmd_vel_publisher_ = this->create_publisher<Twist>("/cmd_vel", 10);

    odom_timer_ = this->create_wall_timer(20ms, std::bind(&EightTrajectory::odom_tf_lookup, this));
    trajectory_timer_ = this->create_wall_timer(20ms, std::bind(&EightTrajectory::execute_trajectory, this));
    logging_timer_ = this->create_wall_timer(500ms, std::bind(&EightTrajectory::print_logging, this));

    wheel_speed_msg_ = Float32MultiArray();
    wheel_speed_msg_.data.resize(4);
    waypoint_idx_ = 0;
    goal_pose_ = OdomPose();
  }

private:
  // rclcpp::Subscription<Odometry>::SharedPtr odom_subscriber_;
  std::shared_ptr<tf2_ros::Buffer> odom_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> odom_tf_listener_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr wheel_speed_publisher_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr trajectory_timer_;
  rclcpp::TimerBase::SharedPtr logging_timer_;
  Float32MultiArray wheel_speed_msg_;
  size_t waypoint_idx_;
  OdomPose odom_pose_;
  OdomPose goal_pose_;
  float wz_, vx_, vy_;

  void odom_tf_lookup() {
    try {
      geometry_msgs::msg::TransformStamped tf = odom_tf_buffer_->lookupTransform("odom", "base_link", rclcpp::Time(0));
      auto q = tf.transform.rotation;
      odom_pose_.set(std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
                     static_cast<float>(tf.transform.translation.x), static_cast<float>(tf.transform.translation.y));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), " ");
      // RCLCPP_WARN(this->get_logger(), "TF lookup odom --> base_link failed: %s", ex.what());
    }
  }

  void execute_trajectory() {
    if (odom_pose_.goal_reached(goal_pose_)) {
      if (waypoint_idx_ < relative_waypoints.size()) {
        // Add relative waypoint to goal pose
        goal_pose_ += OdomPose(relative_waypoints[waypoint_idx_]);
      } else {
        // Finished trajectory --> publish zero twist
        std::fill(wheel_speed_msg_.data.begin(), wheel_speed_msg_.data.end(), 0.0f);
        wheel_speed_publisher_->publish(wheel_speed_msg_);

        odom_timer_->cancel();
        trajectory_timer_->cancel();
        logging_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Finished trajectory!");
        RCLCPP_INFO(this->get_logger(), "========================================");

        return;
      }

      waypoint_idx_++;

      RCLCPP_INFO(this->get_logger(), "========================================");
      RCLCPP_INFO(this->get_logger(), "Going to waypoint %zu --> %s", waypoint_idx_,
                  goal_pose_.to_string().c_str());
      RCLCPP_INFO(this->get_logger(), "Current pose: %s", odom_pose_.to_string().c_str());
      RCLCPP_INFO(this->get_logger(), "========================================");
    }

    // Proportional constants and max values
    const float kp_xy = 2.0;
    const float kp_phi = 2.0;
    const float max_linear = 2.0;
    const float max_angular = 4.0;

    // Get body frame errors
    const float e_x = odom_pose_.error_x(goal_pose_);
    const float e_y = odom_pose_.error_y(goal_pose_);
    const float e_phi = odom_pose_.error_phi(goal_pose_);

    // Apply proportional scaling
    wz_ = std::clamp(kp_phi * e_phi, -max_angular, max_angular);
    vx_ = kp_xy * e_x;
    vy_ = kp_xy * e_y;

    // Clamp linear velocity
    const float v_norm = std::hypot(vx_, vy_);
    if (v_norm > max_linear && v_norm > 1e-6f) {
      const float scale = max_linear / v_norm;
      vx_ *= scale;
      vy_ *= scale;
    }

    // Convert body twist to wheel speeds
    const float v[3] = {wz_, vx_, vy_};
    std::array<float, 4> u{};
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        u[i] += H[i][j] * v[j];
      }
    }

    std::copy(u.begin(), u.end(), wheel_speed_msg_.data.begin());
    wheel_speed_publisher_->publish(wheel_speed_msg_);
  }

  void print_logging() {
    RCLCPP_INFO(this->get_logger(), "Commanded twist: {wz: %.2f, vx: %.2f, vy: %.2f}", wz_, vx_, vy_);
    RCLCPP_INFO(this->get_logger(), "Pose delta: %s", odom_pose_.error_to_string(goal_pose_).c_str());
    RCLCPP_INFO(this->get_logger(), "Pose error: %s", odom_pose_.goal_error_to_string(goal_pose_).c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EightTrajectory>());
  rclcpp::shutdown();
  return 0;
}