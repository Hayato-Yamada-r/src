#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class AutoDriveNode : public rclcpp::Node
{
public:
  AutoDriveNode() : Node("auto_drive")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&AutoDriveNode::publish_drive, this));
  }

private:
  void publish_drive()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 1.0;  // auto drive forward speed
    message.angular.z = 0.0; // no rotation
    publisher_->publish(message);
    // RCLCPP_INFO(this->get_logger(),
    //             "Auto driving: linear.x=%.2f, angular.z=%.2f",
    //             message.linear.x, message.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoDriveNode>());
  rclcpp::shutdown();
  return 0;
}