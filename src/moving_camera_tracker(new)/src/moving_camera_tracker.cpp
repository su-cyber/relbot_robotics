#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class MovingCameraTracker : public rclcpp::Node {
public:
    explicit MovingCameraTracker() 
        : Node("moving_camera_tracker"), max_angle_(90.0) {
        // Subscribe to the camera feed
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/output/moving_camera", 10,
            std::bind(&MovingCameraTracker::image_callback, this, std::placeholders::_1)
        );

        // Publisher for detected object position
        object_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/tracked_object_position", 10
        );

        RCLCPP_INFO(this->get_logger(), "Moving Camera Tracker Node Started! Listening to /output/moving_camera");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat hsv, mask;

            // Convert to HSV and filter green color
            cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
            cv::Scalar lower_green(40, 70, 70);
            cv::Scalar upper_green(90, 255, 255);
            cv::inRange(hsv, lower_green, upper_green, mask);

            // Compute Center of Gravity (CoG) of detected green object
            cv::Moments m = cv::moments(mask, true);
            geometry_msgs::msg::Point object_position;

            if (m.m00 > 0) {
                int cog_x = static_cast<int>(m.m10 / m.m00);
                int cog_y = static_cast<int>(m.m01 / m.m00);

                int image_width = image.cols;
                double theta_z_light = ((cog_x - image_width / 2) * max_angle_) / (image_width / 2);

                object_position.x = cog_x;
                object_position.y = theta_z_light;
                object_position.z = 0.0;

                RCLCPP_INFO(this->get_logger(), "Green object at: (%d, %d), θ: %.2f", cog_x, cog_y, theta_z_light);
            } else {
                object_position.x = -1.0;
                object_position.y = 0.0;
                object_position.z = 0.0;

                RCLCPP_WARN(this->get_logger(), "No green object detected.");
            }

            object_pub_->publish(object_position);

            // Debug: show mask image
            cv::imshow("Green Mask", mask);
            cv::waitKey(1);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr object_pub_;
    const double max_angle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovingCameraTracker>());
    rclcpp::shutdown();
    return 0;
}
