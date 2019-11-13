#include "rclcpp/rclcpp.hpp"
int main(int argc, char* argv[])
{

 rclcpp::init(argc, argv);

 auto node = rclcpp::Node::make_shared("ObiWan");

 RCLCPP_INFO(node->get_logger(), "hello");
 rclcpp::shutdown();
 return 0;
}
