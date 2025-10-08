#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Publish: public rclcpp::Node
{
  public:
  Publish():Node("Pubfirst")
  {
    Pub_ = this->create_publisher<std_msgs::msg::String>("Arknights",10);
    Timer_= this->create_wall_timer(500ms,std::bind(&Publish::Timerback, this));
  }

  private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Pub_;
  rclcpp::TimerBase::SharedPtr Timer_;
  void Timerback()
  {
    std_msgs::msg::String msg;
    msg.data = "Arknights!";
    Pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(),"Success!");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publish>());
  rclcpp::shutdown();
  return 0;
}