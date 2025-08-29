#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>
#include <array>
#include <vector>

using std::placeholders::_1;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Twist = geometry_msgs::msg::Twist;

constexpr float l = 0.085;
constexpr float w = 0.134845;
constexpr float r = 0.05;

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model") {
    wheel_speed_subscriber_ = this->create_subscription<Float32MultiArray>(
        "wheel_speed", 10, std::bind(&KinematicModel::wheel_speed_callback, this, _1));

    cmd_vel_publisher_ = this->create_publisher<Twist>("/cmd_vel", 10);

    Eigen::Matrix<float, 4, 3> H{{(-l - w) / r, 1.0f / r, -1.0f / r},
                                 {(l + w) / r, 1.0f / r, 1.0f / r},
                                 {(l + w) / r, 1.0f / r, -1.0f / r},
                                 {(-l - w) / r, 1.0f / r, 1.0f / r}};

    H_pinv_ = (H.transpose() * H).inverse() * H.transpose();
  }

private:
  rclcpp::Subscription<Float32MultiArray>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
  Twist twist_message_;
  Eigen::Matrix<float, 3, 4> H_pinv_;

  void wheel_speed_callback(const Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 4) {
      RCLCPP_WARN(get_logger(), "Expected 4 wheel speeds, got %zu", msg->data.size());
      return;
    }

    Eigen::Vector4f u = Eigen::Map<const Eigen::Vector4f>(msg->data.data());

    Eigen::Vector3f twist_b = H_pinv_ * u;

    twist_message_.angular.z = twist_b(0);
    twist_message_.linear.x = twist_b(1);
    twist_message_.linear.y = twist_b(2);

    cmd_vel_publisher_->publish(twist_message_);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}