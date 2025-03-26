#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <example_interfaces/msg/float64.hpp>

class Controller123 : public rclcpp::Node {
public:
    explicit Controller123() : Node("controller_1_2_3"), x_set(60.0), theta_z_set(0.0) {
        // Subscribe to object position topic
        object_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/tracked_object_position", 10,
            std::bind(&Controller123::object_callback, this, std::placeholders::_1)
        );

        // Publishers for motor velocities and camera setpoint
        left_motor_pub_ = this->create_publisher<example_interfaces::msg::Float64>("/input/left_motor/setpoint_vel", 10);
        right_motor_pub_ = this->create_publisher<example_interfaces::msg::Float64>("/input/right_motor/setpoint_vel", 10);
        camera_setpoint_pub_ = this->create_publisher<example_interfaces::msg::Float64>("/camera_x_setpoint", 10);

        RCLCPP_INFO(this->get_logger(), "Controller 1.2.3 started.");
    }

private:
    void object_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Invalid object data received.");
            stop_motors();
            return;
        }

        int x_light = msg->data[0];         // x-coordinate of the light in the camera image
        int theta_z_light = msg->data[1];   // theta_z orientation of the light

        if (x_light == -1) { // No object detected
            RCLCPP_WARN(this->get_logger(), "No object detected. Stopping motors.");
            stop_motors();
            return;
        }

        //Error derivatives using a first-order system
        double x_dot = -(x_light - x_set) / TAU;          // Negative sign ensures correct direction
        double theta_dot = -(theta_z_light - theta_z_set) / TAU;

        // Updating setpoints using Forward Euler integration
        x_set += x_dot * DT;
        theta_z_set += theta_dot * DT;

        // Computing motor velocity with proportional control
        double velocity = std::clamp(x_dot * VELOCITY_SCALE, -MAX_VELOCITY, MAX_VELOCITY);

        // Publish motor commands and camera setpoint
        publish_motor_commands(velocity);
        publish_camera_setpoint(x_set);

        RCLCPP_INFO(this->get_logger(), "x_light: %d, x_set: %.3f, x_dot: %.3f, velocity: %.3f",
                    x_light, x_set, x_dot, velocity);
    }

    void publish_motor_commands(double velocity) {
        example_interfaces::msg::Float64 motor_msg;
        motor_msg.data = velocity;

        // Publishing the same velocity to both motors for forward motion
        left_motor_pub_->publish(motor_msg);
        right_motor_pub_->publish(motor_msg);
    }

    void publish_camera_setpoint(double x_set) {
        example_interfaces::msg::Float64 camera_msg;
        camera_msg.data = x_set;
        camera_setpoint_pub_->publish(camera_msg);
    }

    void stop_motors() {
        example_interfaces::msg::Float64 stop_msg;
        stop_msg.data = 0.0;

        left_motor_pub_->publish(stop_msg);
        right_motor_pub_->publish(stop_msg);
    }

    // Subscribers and publishers
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr object_sub_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_motor_pub_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr camera_setpoint_pub_;

    // Constants for control logic
    const double TAU = 2.0;           // Time constant for smoothing
    const double DT = 0.05;           // Integration step size
    const double VELOCITY_SCALE = 0.5; // Scaling factor for motor velocity
    const double MAX_VELOCITY = 5.0;  // Maximum allowable motor velocity

    // State variables
    double x_set;       
    double theta_z_set; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller123>());
    rclcpp::shutdown();
    return 0;
}