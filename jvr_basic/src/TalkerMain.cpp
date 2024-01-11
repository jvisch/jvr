#include "jvr_basic/TalkerNode.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jvr::basic::TalkerNode>());
  rclcpp::shutdown();
  return 0;
}