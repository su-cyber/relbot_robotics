#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"

class ResponseNode07 : public rclcpp::Node {
public:
    ResponseNode07() : Node("response_07") {
        rclcpp::QoS qos_settings = rclcpp::QoS(10)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL)
            .keep_all();

        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "msg_out_seq07", qos_settings, std::bind(&ResponseNode07::handle_incoming_message, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::String>("msg_in_seq07", qos_settings);

        RCLCPP_INFO(this->get_logger(), "ResponseNode07 is active and waiting for messages.");
    }

private:
    void handle_incoming_message(const std_msgs::msg::String::SharedPtr msg) {
        auto current_time = this->now().nanoseconds();
        auto data_parts = parse_message(msg->data);

        if (data_parts.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Received malformed message: '%s'", msg->data.c_str());
            return;
        }

        auto response_msg = std_msgs::msg::String();
        response_msg.data = data_parts[0] + "," + std::to_string(current_time); // Return same ID with new timestamp
        publisher_->publish(response_msg);

        RCLCPP_INFO(this->get_logger(), "✅ Echoing back message ID %s", data_parts[0].c_str());
    }

    std::vector<std::string> parse_message(const std::string &msg) {
        std::vector<std::string> parsed;
        std::stringstream ss(msg);
        std::string item;
        while (std::getline(ss, item, ',')) {
            parsed.push_back(item);
        }
        return parsed;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ResponseNode07>());
    rclcpp::shutdown();
    return 0;
}
