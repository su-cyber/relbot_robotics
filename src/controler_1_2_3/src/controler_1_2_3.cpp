#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"
#include <algorithm>

class RelbotMotorController : public rclcpp::Node
{
public:
  RelbotMotorController()
    : Node("controler_1_2_3"),
      tau_(2.0),
      max_velocity_(5.0),
      velocity_scale_(0.05),
      turn_scale_(0.02),
      // Search behavior variables (currently not used because we only want straight backward movement)
      search_ticks_(0),
      search_direction_(1),
      // State variables
      object_x_(0.0),
      object_theta_(0.0),
      target_valid_(false)
  {
    object_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/tracked_object_position", 10,
      std::bind(&RelbotMotorController::object_callback, this, std::placeholders::_1));

      publisher_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&RelbotMotorController::update_loop, this));

    RCLCPP_INFO(this->get_logger(), "Relbot motor controller initialized.");
  }

private:
  // Callback for when a new object position is received
  void object_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[OBJECT_CB] Received x: %.2f, y: %.2f", msg->x, msg->y);
    // If the object is not detected, msg->x is -1.0.
    if (msg->x == -1.0) {
      target_valid_ = false;
    } else {
      object_x_ = msg->x;
      object_theta_ = msg->y;
      target_valid_ = true;
    }
  }

  // Main loop called every 50ms
  void update_loop()
  {
    if (target_valid_) {
      perform_tracking();
    } else {
      perform_searching();
    }
    // Reset target validity for the next loop
    target_valid_ = false;
  }

  // When an object is visible, track it
  void perform_tracking()
  {
    double x_error = desired_x_ - object_x_;
    double theta_error = -object_theta_;

    double forward_vel = std::clamp(x_error * velocity_scale_, -max_velocity_, max_velocity_);
    double turn_vel = std::clamp(theta_error * turn_scale_, -max_velocity_, max_velocity_);

    double left_vel = forward_vel - turn_vel;
    double right_vel = forward_vel + turn_vel;

    publish_motors(left_vel, right_vel);

    RCLCPP_INFO(this->get_logger(),
                "[TRACKING] x: %.2f, θ: %.2f | Forward: %.2f, Turn: %.2f → Left: %.2f, Right: %.2f",
                object_x_, object_theta_, forward_vel, turn_vel, left_vel, right_vel);
  }

  // When no object is detected, command a simple constant backward movement.
  void perform_searching()
  {
    // Instead of zig-zag logic, simply command a constant backward speed.
    double left_vel = -2.0;
    double right_vel = -2.0;

    // (The old zig-zag logic is commented out below)
    /*
    double turn = search_direction_ * search_turn_speed_;
    double left_vel = -search_speed_ - turn;
    double right_vel = -search_speed_ + turn;
    search_ticks_++;
    if (search_ticks_ >= max_search_ticks_) {
      search_ticks_ = 0;
      search_direction_ *= -1;
    }
    */

    publish_motors(left_vel, right_vel);

    RCLCPP_WARN(this->get_logger(), "[SEARCHING] STRAIGHT BACKWARDS → Left: %.2f, Right: %.2f",
                left_vel, right_vel);
  }

  // Publish the motor commands to the respective topics
  void publish_motors(double left, double right)
  {
    auto msg = xrf2_msgs::msg::Ros2Xeno();
    msg.left_motor = left;
    msg.right_motor = right;

    publisher_->publish(msg);
  }

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr object_sub_;
  rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Control parameters (order matters: declared in the same order as in the initializer list)
  const double tau_;
  const double max_velocity_;
  const double velocity_scale_;
  const double turn_scale_;
  const double desired_x_ = 60.0;

  // Search behavior parameters (these are currently not used because we only command backward movement)
  int search_ticks_;
  int search_direction_;
  const int max_search_ticks_ = 10;
  const double search_speed_ = 2.0;       // previously set; now not used
  const double search_turn_speed_ = 1.0;     // previously set; now not used

  // State variables
  double object_x_;
  double object_theta_;
  bool target_valid_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelbotMotorController>());
  rclcpp::shutdown();
  return 0;
}
