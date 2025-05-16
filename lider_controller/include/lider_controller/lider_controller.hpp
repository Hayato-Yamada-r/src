#ifndef LIDER_CONTROLLER_HPP_
#define LIDER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include "nav2_core/controller.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/goal_checker.hpp"

namespace lider_controller {

class LiderController : public nav2_core::Controller
{
public:
  LiderController();
  ~LiderController() override;

  // Configure the controller using the proper interface signature:
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // Set the global plan
  void setPlan(const nav_msgs::msg::Path & path) override;

  // Correct signature for computeVelocityCommands:
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & current_velocity,
      nav2_core::GoalChecker * goal_checker) override;

  // Set a speed limit (dummy implementation)
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  nav_msgs::msg::Path global_plan_;
};

}  // namespace lider_controller

#endif  // LIDER_CONTROLLER_HPP_