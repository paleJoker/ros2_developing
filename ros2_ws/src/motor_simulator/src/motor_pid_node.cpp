#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class MotorPub:public rclcpp::Node
{
    public:
    MotorPub():Node("motor_pub")
    {
        control_torque_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "motor_simulator_control_input", 10);
        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "simulated_motor_velocity",10,
            std::bind(&MotorPub::velocity_filter,this,std::placeholders::_1));
        true_velocity_ = this->create_publisher<std_msgs::msg::Float64>(
            "velocity_filter", 10);
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&MotorPub::motor_pid, this));
        RCLCPP_INFO(get_logger(), "motor pid launch");
    }


    private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_torque_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr true_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double target = 350.0;
    double integral = 0.0;
    double lastoutput = 0.0;
    double lasterror = 0.0;
    double current = 0.0;
    double alpha = 0.2;
    double kp = 0.8;
    double ki = 0.15;
    double kd = 0.05;
    double k_torque = 1.0;

    void velocity_filter(std_msgs::msg::Float64 msg)
    {   
        current = msg.data;
        current = alpha *current + (1- alpha) * lastoutput;
        lastoutput = current;
        msg.data = current;
        true_velocity_->publish(msg);
    }
    void motor_pid()
    {
        std_msgs::msg::Float64 msg;
        double error  = target - current;
        double output_p = kp * error;

        integral += ki * error * 0.001;
        double output_i = integral;

        double output_d = kd * (error - lasterror) /0.001;
        lasterror = error;

        msg.data = k_torque * (output_p + output_i + output_d);
        control_torque_pub_->publish(msg);
    }
};



int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MotorPub>());
    rclcpp::shutdown();
}