import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from custom_interfaces.msg import WifiRaw, WifiFused


class WifiFusionNode(Node):
    def __init__(self):
        super().__init__('wifi_fusion_node')

        self.latest_grid = None

        self.create_subscription(
            WifiRaw,
            '/wifi/raw',
            self.wifi_callback,
            10
        )

        self.create_subscription(
            Point,
            '/grid_pose',
            self.grid_callback,
            10
        )

        self.publisher = self.create_publisher(
            WifiFused,
            '/wifi/fused',
            10
        )

        self.get_logger().info("WiFi Fusion Node started")

    def grid_callback(self, msg):
        self.latest_grid = (int(msg.x), int(msg.y))

    def wifi_callback(self, msg):
        if self.latest_grid is None:
            return

        fused = WifiFused()
        fused.grid_x = self.latest_grid[0]
        fused.grid_y = self.latest_grid[1]
        fused.ssid = msg.ssid
        fused.rssi = msg.rssi
        fused.stamp = msg.stamp

        self.publisher.publish(fused)


def main():
    rclpy.init()
    rclpy.spin(WifiFusionNode())
    rclpy.shutdown()

