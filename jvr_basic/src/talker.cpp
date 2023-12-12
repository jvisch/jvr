#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "jvr_interfaces/msg/talk_msg.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Talker : public rclcpp::Node
{
public:
  Talker() : Node("Talker"), count_(0)
  {
    publisher_ = this->create_publisher<jvr_interfaces::msg::TalkMsg>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = jvr_interfaces::msg::TalkMsg();
    message.id = count_++;
    message.content = "Publishing " + std::to_string(message.id);
    RCLCPP_INFO(this->get_logger(), message.content.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<jvr_interfaces::msg::TalkMsg>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}