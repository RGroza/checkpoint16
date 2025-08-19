#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <memory>

using namespace std::chrono_literals;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher") {
    wheel_speed_publisher_ = this->create_publisher<Float32MultiArray>("/wheel_speed", 10);

    publish_wheel_velocities();
  }

private:
  rclcpp::Publisher<Float32MultiArray>::SharedPtr wheel_speed_publisher_;

  void publish_speed(std::vector<float> velocities, std::string logging_string) {
    Float32MultiArray wheel_speed_message;
    wheel_speed_message.data = velocities;
    auto start = this->get_clock()->now();
    auto limit = rclcpp::Duration::from_seconds(3);
    rclcpp::WallRate rate(10);

    RCLCPP_INFO(this->get_logger(), "%s", logging_string.c_str());

    while (rclcpp::ok() && (this->get_clock()->now() - start) < limit) {
      wheel_speed_publisher_->publish(wheel_speed_message);
      rate.sleep();
    }
  }

  void publish_wheel_velocities() {
    std::vector<std::pair<std::vector<float>, std::string>> wheel_velocities_list = {
        {{+1.0, +1.0, +1.0, +1.0}, "Move forward"},
        {{-1.0, -1.0, -1.0, -1.0}, "Move backward"},
        {{-1.0, +1.0, -1.0, +1.0}, "Move left"},
        {{+1.0, -1.0, +1.0, -1.0}, "Move right"},
        {{+1.0, -1.0, -1.0, +1.0}, "Turn clockwise"},
        {{-1.0, +1.0, +1.0, -1.0}, "Turn counter-clockwise"},
        {{0.0, 0.0, 0.0, 0.0}, "Stop"}};

    for (const auto& velocities : wheel_velocities_list) {
      publish_speed(velocities.first, velocities.second);
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<WheelVelocitiesPublisher> wheel_velocities_publisher = std::make_shared<WheelVelocitiesPublisher>();
  rclcpp::shutdown();
  return 0;
}