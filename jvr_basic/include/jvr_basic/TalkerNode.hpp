#include "jvr_interfaces/msg/talk_msg.hpp"

#include "rclcpp/rclcpp.hpp"

class TalkerNode : public rclcpp::Node
{
public:
  TalkerNode();

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<jvr_interfaces::msg::TalkMsg>::SharedPtr publisher_;
  size_t count_;
};
