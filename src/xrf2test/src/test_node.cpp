#include <rclcpp/rclcpp.hpp>
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using std::placeholders::_1;

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("test_node")
  {
    publisher_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&TestPublisher::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    auto msg = xrf2_msgs::msg::Ros2Xeno();
    msg.header.stamp = this->get_clock()->now();
    msg.left_motor = 0.3;
    msg.right_motor = 0.3;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sent command: L=%.2f, R=%.2f", msg.left_motor, msg.right_motor);
  }

  rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPublisher>());
  rclcpp::shutdown();
  return 0;
}
