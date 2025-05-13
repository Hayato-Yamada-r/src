import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from transforms3d.euler import quat2euler  # 変更点

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('auto_drive_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.target_position = [2.0, 2.0]  # 目標位置 (x, y)
        self.current_position = [0.0, 0.0]
        self.current_orientation = 0.0

    def odom_callback(self, msg):
        # 現在位置と向きを取得
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.quaternion_to_euler(orientation_q)

        # デバッグログ追加
        self.get_logger().info(
            f"Odometry received: position=({self.current_position[0]:.2f}, {self.current_position[1]:.2f}), orientation={self.current_orientation:.2f}"
        )

        # 自動走行ロジック
        self.navigate_to_target()

    def quaternion_to_euler(self, q):
        # transforms3dのquat2eulerを使ってクォータニオンをオイラー角に変換
        return quat2euler([q.x, q.y, q.z, q.w])  # デフォルトのaxesは'sxyz'

    def navigate_to_target(self):
        # 目標位置までの距離と角度を計算
        dx = self.target_position[0] - self.current_position[0]
        dy = self.target_position[1] - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        # 回転と直進の制御
        angle_diff = target_angle - self.current_orientation
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # 正規化

        twist = Twist()
        if distance > 0.1:  # 目標位置に近づくまで移動
            if abs(angle_diff) > 0.1:  # 目標方向に回転
                twist.angular.z = 0.5 * angle_diff
            else:  # 目標方向に直進
                twist.linear.x = 0.5 * distance
        else:
            # 目標位置に到達：停止してログ出力
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Target reached. Stopping.")
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()