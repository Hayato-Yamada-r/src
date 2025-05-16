#ifndef INROF_CMD_VEL_PUBLISHER_HPP_
#define INROF_CMD_VEL_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher();

private:
    void publish_cmd_vel();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // INROF_CMD_VEL_PUBLISHER_HPP_