#include "jvr_basic/ListenerNode.hpp"

using namespace jvr::basic;

ListenerNode::ListenerNode() : Node("listener_node")
{
    auto cb = [this](const jvr_interfaces::msg::TalkMsg &msg)
    {
        callback(msg);
    };

    subscription_ = this->create_subscription<jvr_interfaces::msg::TalkMsg>(
        "italk/talk", 10, cb);
}

void ListenerNode::callback(const jvr_interfaces::msg::TalkMsg &msg)
{
    RCLCPP_INFO(this->get_logger(), "content: '%s', id '%lu'", msg.content.c_str(), msg.id);
}
