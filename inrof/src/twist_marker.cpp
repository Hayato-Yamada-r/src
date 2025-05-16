#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"

class TwistMarker : public rclcpp::Node
{
public:
  TwistMarker() : Node("twist_marker")
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("twist_marker", 10);
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&TwistMarker::twist_callback, this, std::placeholders::_1)
    );
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Markerをセット
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cmd_vel";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Markerの大きさ・色の設定
    marker.scale.x = 0.5;  // 矢印の長さ
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    // 原点からcmd_velのlinear.x, angular.z（例として）を利用して矢印の終点を設定
    marker.points.resize(2);
    marker.points[0].x = 0.0;
    marker.points[0].y = 0.0;
    marker.points[0].z = 0.0;
    marker.points[1].x = msg->linear.x;  // 直進速度に比例
    marker.points[1].y = msg->angular.z; // 回転速度に比例（例示用）
    marker.points[1].z = 0.0;

    marker_pub_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistMarker>());
  rclcpp::shutdown();
  return 0;
}