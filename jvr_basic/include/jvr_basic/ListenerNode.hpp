#include "jvr_interfaces/msg/talk_msg.hpp"

#include "rclcpp/rclcpp.hpp"

namespace jvr
{
    namespace basic
    {
        class ListenerNode : public rclcpp::Node
        {
        public:
            ListenerNode();

        private:
            void callback(const jvr_interfaces::msg::TalkMsg &msg);

            rclcpp::Subscription<jvr_interfaces::msg::TalkMsg>::SharedPtr subscription_;
        };
    }
}