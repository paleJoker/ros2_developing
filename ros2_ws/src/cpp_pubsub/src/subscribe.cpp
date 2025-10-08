#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

class Subscribe: public rclcpp::Node
{
  public:
  Subscribe():Node("Subfirst")
  {
    Sub_ = this->create_subscription<std_msgs::msg::String>("Arknights",10,std::bind(&Subscribe::SubBack,this,std::placeholders::_1));
  }
  
  private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Sub_;
  void SubBack(const std_msgs::msg::String &msg)
  {
    RCLCPP_INFO(this->get_logger(),"'%s',For the better world!",msg.data.c_str());
  }
};

int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Subscribe>());
  rclcpp::shutdown();
  return 0;
}
