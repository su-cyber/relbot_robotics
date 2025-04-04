#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class MovingCameraTracker : public rclcpp::Node {
public:
    explicit MovingCameraTracker() 
        : Node("moving_camera_tracker"), max_angle_(90.0) {
        // HSV parameters for green color detection
        declare_parameters();
        
        // Subscribe to the camera feed
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/output/moving_camera", 10,
            std::bind(&MovingCameraTracker::image_callback, this, std::placeholders::_1)
        );

        // Publisher for detected object position
        object_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/tracked_object_position", 10
        );

        // Parameter update callback
        param_cb_ = this->add_on_set_parameters_callback(
            std::bind(&MovingCameraTracker::param_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Moving Camera Tracker Node Started! Listening to /output/moving_camera");
    }

private:
    void declare_parameters() {
        // Default HSV range for green color
        this->declare_parameter("h_low", 35);
        this->declare_parameter("h_high", 85);
        this->declare_parameter("s_low", 50);
        this->declare_parameter("s_high", 255);
        this->declare_parameter("v_low", 50);
        this->declare_parameter("v_high", 255);

        // Get initial values
        this->get_parameter("h_low", h_low_);
        this->get_parameter("h_high", h_high_);
        this->get_parameter("s_low", s_low_);
        this->get_parameter("s_high", s_high_);
        this->get_parameter("v_low", v_low_);
        this->get_parameter("v_high", v_high_);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat hsv, mask;
            
            // Convert to HSV color space
            cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
            
            // Create color mask
            cv::inRange(hsv, 
                cv::Scalar(h_low_, s_low_, v_low_),
                cv::Scalar(h_high_, s_high_, v_high_),
                mask);

            // Noise reduction
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::erode(mask, mask, kernel);
            cv::dilate(mask, mask, kernel);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            std_msgs::msg::Int32MultiArray object_position;
            int cog_x = -1;
            double theta_z_light = -1;

            if (!contours.empty()) {
                // Find largest contour
                auto largest_contour = std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) < cv::contourArea(b);
                    });

                // Calculate centroid
                cv::Moments m = cv::moments(*largest_contour);
                if (m.m00 > 10) {  // Ignore small contours
                    cog_x = static_cast<int>(m.m10 / m.m00);
                    int cog_y = static_cast<int>(m.m01 / m.m00);
                    
                    // Calculate theta_z_light
                    int image_width = image.cols;
                    theta_z_light = ((cog_x - image_width / 2) * max_angle_) / (image_width / 2);
                    
                    RCLCPP_INFO(this->get_logger(), "Object Detected at: (%d, %d), theta_z_light: %.2f", 
                              cog_x, cog_y, theta_z_light);
                }
            }

            object_position.data = {cog_x, static_cast<int>(theta_z_light)};
            object_pub_->publish(object_position);

            // Debug: Show mask image
            cv::imshow("Detection Mask", mask);
            cv::waitKey(1);

        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
            const std::string& name = param.get_name();
            int value = param.as_int();

            if (name == "h_low") h_low_ = value;
            else if (name == "h_high") h_high_ = value;
            else if (name == "s_low") s_low_ = value;
            else if (name == "s_high") s_high_ = value;
            else if (name == "v_low") v_low_ = value;
            else if (name == "v_high") v_high_ = value;

            RCLCPP_INFO(this->get_logger(), "Updated %s: %d", name.c_str(), value);
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    // ROS2 Communication Objects
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr object_pub_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // HSV parameters
    int h_low_, h_high_;
    int s_low_, s_high_;
    int v_low_, v_high_;
    const double max_angle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovingCameraTracker>());
    rclcpp::shutdown();
    return 0;
}