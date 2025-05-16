#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node
{
public:
  CmdVelPublisher() : Node("cmd_vel_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&CmdVelPublisher::publish_cmd_vel, this));
  }

private:
  void publish_cmd_vel()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.5;   // forward speed
    message.angular.z = 0.1;  // angular speed
    publisher_->publish(message);
    // RCLCPP_INFO(this->get_logger(),
    //             "Publishing cmd_vel: linear.x=%.2f, angular.z=%.2f",
    //             message.linear.x, message.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelPublisher>());
  rclcpp::shutdown();
  return 0;
}