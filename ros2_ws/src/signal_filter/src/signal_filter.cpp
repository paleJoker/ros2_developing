#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class SignalFilter: public rclcpp::Node
{
    public:
    SignalFilter():Node("signal_filter")
    {
        Median_Signal_Sub_ = this->create_subscription<std_msgs::msg::Float64>("SignalFilter",
            10,
            std::bind(&SignalFilter::Median_Filter,this,std::placeholders::_1));
        Lowpass_Signal_Sub_ = this->create_subscription<std_msgs::msg::Float64>("SignalFilter",
            10,
            std::bind(&SignalFilter::Lowpass_Filter,this,std::placeholders::_1));
        Median_Filter_Pub_ = this->create_publisher<std_msgs::msg::Float64>("Median_Filter_Pub",
            10);
        Lowpass_Filter_Pub_ = this->create_publisher<std_msgs::msg::Float64>("Lowpass_Filter_Pub",10);
        RCLCPP_INFO(this->get_logger(),"signal_filter已启动");
    }
    private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Median_Signal_Sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Lowpass_Signal_Sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Median_Filter_Pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Lowpass_Filter_Pub_;
    double Median_Temp[3] = {0.0};
    double PreOut = 0.0;
    double alpha = 0.8;
    void Median_Filter(std_msgs::msg::Float64 msg)
    {   
        std_msgs::msg::Float64 Median_msg;
        for(int i =0;i<2;i++)
        {
            Median_Temp[i] = Median_Temp[i+1];
        }
        Median_Temp[2] = msg.data;
        if(Median_Temp[0] >= Median_Temp[1] && Median_Temp[0] <= Median_Temp[2])
        Median_msg.data = Median_Temp[0];
        else if(Median_Temp[1] >= Median_Temp[0] && Median_Temp[1] <= Median_Temp[2])
        Median_msg.data = Median_Temp[1];
        else
        Median_msg.data = Median_Temp[2];
        Median_Filter_Pub_->publish(Median_msg);
    }
    void Lowpass_Filter(std_msgs::msg::Float64 msg)
    {
        std_msgs::msg::Float64 Lowpass_msg; 
        if(msg.data - PreOut < 1.5 || msg.data - PreOut > -1.5)
        {   
            Lowpass_msg.data = alpha * msg.data + (1 - alpha) * PreOut;
            PreOut = Lowpass_msg.data;
        }
        else
        {
            Lowpass_msg.data = msg.data;
            PreOut = Lowpass_msg.data;
        }
        Lowpass_Filter_Pub_->publish(Lowpass_msg);
    }
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SignalFilter>());
    rclcpp::shutdown();
}