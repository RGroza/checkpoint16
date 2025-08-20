#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <array>
#include <vector>

using std::placeholders::_1;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;
using Twist = geometry_msgs::msg::Twist;

constexpr float l = 0.085;
constexpr float w = 0.26969;
constexpr float r = 0.05;
constexpr float c_wz = r / (4.0f * (l + w));
constexpr float c_v = r / 4.0f;

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model") {
    wheel_speed_subscriber_ = this->create_subscription<Float32MultiArray>(
        "wheel_speed", 10, std::bind(&KinematicModel::wheel_speed_callback, this, _1));

    cmd_vel_publisher_ = this->create_publisher<Twist>("/cmd_vel", 10);
  }

private:
  rclcpp::Subscription<Float32MultiArray>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;

  void wheel_speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 4) {
      RCLCPP_WARN(get_logger(), "Expected 4 wheel speeds, got %zu", msg->data.size());
      return;
    }

    const auto& u = msg->data;

    Twist twist_message = Twist();
    twist_message.angular.z = c_wz * (-u[0] + u[1] + u[2] - u[3]);
    twist_message.linear.x = c_v * (u[0] + u[1] + u[2] + u[3]);
    twist_message.linear.y = c_v * (-u[0] + u[1] - u[2] + u[3]);

    cmd_vel_publisher_->publish(twist_message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}