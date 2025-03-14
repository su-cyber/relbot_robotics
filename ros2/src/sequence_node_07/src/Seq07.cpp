#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

using namespace std::chrono_literals;

class SequenceNode07 : public rclcpp::Node {
public:
    SequenceNode07() : Node("sequence_07"), experiment_duration_(20.0) {
        this->declare_parameter("queue_size", 10);
        int queue_size = this->get_parameter("queue_size").as_int();

        rclcpp::QoS qos_profile = rclcpp::QoS(queue_size)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL)
            .keep_all();

        publisher_ = this->create_publisher<std_msgs::msg::String>("msg_out_seq07", qos_profile);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "msg_in_seq07", qos_profile, std::bind(&SequenceNode07::message_callback, this, std::placeholders::_1));

        log_file_.open("experiment_seq07.csv");
        log_file_ << "Index,RTT(ms),Jitter(ms),RTT_Jitter(ms)\n";

        experiment_start_time_ = this->now().seconds();
        prev_rtt_ = 0.0;
        prev_jitter_ = 0.0;
        msg_counter_ = 0;

        RCLCPP_INFO(this->get_logger(), "SequenceNode07 started, sending messages every 1ms for 20 seconds...");

        // Start timer immediately
        timer_ = this->create_wall_timer(1ms, std::bind(&SequenceNode07::send_message, this));
    }

private:
    void send_message() {
        double elapsed_time = this->now().seconds() - experiment_start_time_;
        if (elapsed_time >= experiment_duration_) {
            conclude_experiment();
            return;
        }

        auto msg = std_msgs::msg::String();
        auto timestamp = this->now().nanoseconds();
        msg.data = std::to_string(msg_counter_) + "," + std::to_string(timestamp);
        
        publisher_->publish(msg);
        msg_timestamps_[msg_counter_] = timestamp; // Store timestamp for RTT calculation
        
        msg_counter_++;
    }

    void message_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto current_time = this->now().nanoseconds();
        auto data_parts = parse_message(msg->data);

        if (data_parts.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Received malformed message: '%s'", msg->data.c_str());
            return;
        }

        try {
            int received_id = std::stoi(data_parts[0]);
            long original_timestamp = std::stoll(data_parts[1]);

            double round_trip_time = (current_time - original_timestamp) / 1e6;
            double jitter = (prev_rtt_ == 0.0) ? 0.0 : (round_trip_time - prev_rtt_);
            double rtt_jitter = (prev_jitter_ == 0.0) ? 0.0 : (jitter - prev_jitter_);

            prev_rtt_ = round_trip_time;
            prev_jitter_ = jitter;

            log_file_ << received_id << "," << round_trip_time << "," << jitter << "," << rtt_jitter << "\n";

            RCLCPP_INFO(this->get_logger(), "✅ Received response for ID %d with RTT: %.3f ms", received_id, round_trip_time);

        } catch (const std::invalid_argument &e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Invalid message format (stoi failed): %s", msg->data.c_str());
        }
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

    void conclude_experiment() {
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "SequenceNode07: Experiment complete after 20 seconds. Shutting down.");
        rclcpp::shutdown();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ofstream log_file_;

    double experiment_start_time_;
    double experiment_duration_;
    int msg_counter_;
    double prev_rtt_;
    double prev_jitter_;
    std::map<int, long> msg_timestamps_; // Store timestamps of sent messages
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceNode07>());
    rclcpp::shutdown();
    return 0;
}
