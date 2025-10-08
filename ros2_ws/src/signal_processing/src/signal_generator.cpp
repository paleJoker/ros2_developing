#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>


class SignalGenerator : public rclcpp::Node
{
public:
    SignalGenerator() : Node("signal_generator")
    {
        sin_pub_ = this->create_publisher<std_msgs::msg::Float64>("sin_signal", 10);
        square_pub_ = this->create_publisher<std_msgs::msg::Float64>("square_signal", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&SignalGenerator::PublishSignals, this)
        );

        RCLCPP_INFO(this->get_logger(), "分别生成正弦波和方波: sin=10Hz, square=1Hz");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sin_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr square_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void PublishSignals()
    {
        auto t = this->now().seconds();
        std_msgs::msg::Float64 sin_msg;
        sin_msg.data = sin(2 * M_PI * 10 * t);
        sin_pub_->publish(sin_msg);

        std_msgs::msg::Float64 square_msg;
        square_msg.data = (sin(2 * M_PI * 1 * t) >= 0) ? 1.0 : -1.0;
        square_pub_->publish(square_msg);
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalGenerator>());
    rclcpp::shutdown();
    return 0;
}
