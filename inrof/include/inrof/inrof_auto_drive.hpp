#ifndef INROF_AUTO_DRIVE_HPP_
#define INROF_AUTO_DRIVE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class AutoDriveNode : public rclcpp::Node
{
public:
    AutoDriveNode();

private:
    void drive();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // INROF_AUTO_DRIVE_HPP_