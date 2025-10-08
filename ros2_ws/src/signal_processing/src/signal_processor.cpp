#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SignalProcessor : public rclcpp::Node
{
public:
    SignalProcessor() : Node("signal_processor")
    {
        sin_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "sin_signal",
            10,
            std::bind(&SignalProcessor::SinCallback, this, std::placeholders::_1)
        );

        square_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "square_signal",
            10,
            std::bind(&SignalProcessor::SquareCallback, this, std::placeholders::_1)
        );

        processed_pub_ = this->create_publisher<std_msgs::msg::Float64>("processed_signal", 10);

        RCLCPP_INFO(this->get_logger(), "启动Signal_processor");
    }

private:
    double sin_ = 0.0;
    double square_ = 0.0;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sin_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr square_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr processed_pub_;

    void SinCallback(const std_msgs::msg::Float64 & msg)
    {
        sin_ = msg.data;
        ProcessPublish();
    }

    void SquareCallback(const std_msgs::msg::Float64 & msg)
    {
        square_ = msg.data;
        ProcessPublish();
    }

    void ProcessPublish()
    {
        std_msgs::msg::Float64 out_msg;
        if ((sin_ >= 0 && square_ >= 0) || (sin_ <= 0 && square_ <= 0))
        {
            out_msg.data = sin_; 
        } 
        else 
        {
            out_msg.data = 0.0;       
        }
        processed_pub_->publish(out_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}
