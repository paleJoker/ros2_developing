#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <random>
#include <cmath>

class SignalPublish: public rclcpp::Node
{   
    public:
    SignalPublish():Node("signal_publish")
    {
        Signal_ = this->create_publisher<std_msgs::msg::Float64>("SignalFilter",10);
        Timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SignalPublish::SignalPub, this));
        random_gen_ = std::mt19937(std::random_device{}());
        signal.data = -2.0;
        RCLCPP_INFO(this->get_logger(),"已生成包含噪声的信号");
    }

    private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Signal_;
    rclcpp::TimerBase::SharedPtr Timer_;
    std::mt19937 random_gen_;
    std_msgs::msg::Float64 signal;
    int flag = 0;
    double noise_min = -0.2;
    double noise_max = 0.2;

    void SignalPub()
    {
        std::uniform_real_distribution<double> noise_dist(noise_min, noise_max);
        double random_noise = noise_dist(random_gen_); 
        if(flag!=10)
        {
            signal.data+=0.4;
            flag++;
        }
        else
        {
            flag = 0;
            signal.data = -2.0;
        }
        signal.data+=random_noise;
        Signal_->publish(signal);
  
    }

};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SignalPublish>());
    rclcpp::shutdown();
}