#include "lider_controller/lider_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace lider_controller {

LiderController::LiderController()
{
}

LiderController::~LiderController()
{
}

void LiderController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  node_ = parent;
  if (auto node = parent.lock()) {
    RCLCPP_INFO(node->get_logger(), "LiderController [%s] configured", name.c_str());
  }
}

void LiderController::cleanup()
{
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "LiderController cleanup");
  }
}

void LiderController::activate()
{
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "LiderController activated");
  }
}

void LiderController::deactivate()
{
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "LiderController deactivated");
  }
}

void LiderController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "Plan set with %ld poses", path.poses.size());
  }
}

geometry_msgs::msg::TwistStamped LiderController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & current_velocity,
    nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  // --- Dummy Implementation ---
  // Replace this with your velocity command generator (path following) algorithm.
  cmd_vel.twist.linear.x = 0.5;
  cmd_vel.twist.angular.z = 0.0;

  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "Computing velocity command");
  }
  return cmd_vel;
}

void LiderController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "Setting speed limit: %f, percentage: %s",
                speed_limit, percentage ? "true" : "false");
  }
}

}  // namespace lider_controller

PLUGINLIB_EXPORT_CLASS(lider_controller::LiderController, nav2_core::Controller)