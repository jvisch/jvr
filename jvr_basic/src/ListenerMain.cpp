#include "jvr_basic/ListenerNode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<jvr::basic::ListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
