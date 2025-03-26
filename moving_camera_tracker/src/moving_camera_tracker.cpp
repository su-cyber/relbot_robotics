#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
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
        object_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/tracked_object_position", 10
        );

        RCLCPP_INFO(this->get_logger(), "Moving Camera Tracker Node Started! Listening to /output/moving_camera");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat gray, thresholded;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, thresholded, 200, 255, cv::THRESH_BINARY);

            // Compute Center of Gravity (CoG) of detected object
            cv::Moments m = cv::moments(thresholded, true);
            std_msgs::msg::Int32MultiArray object_position;

            if (m.m00 > 0) {
                int cog_x = static_cast<int>(m.m10 / m.m00);
                int cog_y = static_cast<int>(m.m01 / m.m00);

                // Calculate theta_z_light (normalized to the camera's FOV)
                int image_width = image.cols;
                double theta_z_light = ((cog_x - image_width / 2) * max_angle_) / (image_width / 2);

                object_position.data = {cog_x, static_cast<int>(theta_z_light)};
                RCLCPP_INFO(this->get_logger(), "Object Detected at: (%d, %d), theta_z_light: %.2f", cog_x, cog_y, theta_z_light);
            } else {
                object_position.data = {-1, -1}; 
                RCLCPP_WARN(this->get_logger(), "No object detected.");
            }

            // Publish object position and theta_z_light
            object_pub_->publish(object_position);

            // Debug: Show thresholded image
            cv::imshow("Thresholded", thresholded);
            cv::waitKey(1);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr object_pub_;


    const double max_angle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovingCameraTracker>());
    rclcpp::shutdown();
    return 0;
}