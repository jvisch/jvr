#include "jvr_basic/TalkerNode.hpp"

using namespace std::chrono_literals;

TalkerNode::TalkerNode() : Node("talker_node"), count_(0)
{
  publisher_ = this->create_publisher<jvr_interfaces::msg::TalkMsg>("italk/talk", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&TalkerNode::timer_callback, this));
}

void TalkerNode::timer_callback()
{
  auto message = jvr_interfaces::msg::TalkMsg();
  message.id = count_++;
  message.content = "Publishing " + std::to_string(message.id);
  RCLCPP_INFO(this->get_logger(), message.content.c_str());
  publisher_->publish(message);
}
