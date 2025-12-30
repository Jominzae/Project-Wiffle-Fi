import rclpy
from rclpy.node import Node
from wifi_interface.msg import WifiRssi, WifiFused
from geometry_msgs.msg import Point

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.sub_rssi = self.create_subscription(
            WifiRssi, '/wifi/rssi', self.rssi_cb, 10)
        self.sub_pose = self.create_subscription(
            Point, '/pose/dummy', self.pose_cb, 10)

        self.pub = self.create_publisher(WifiFused, '/wifi/fused', 10)

        self.last_rssi = None
        self.last_pose = None

    def rssi_cb(self, msg):
        self.last_rssi = msg
        self.try_publish()

    def pose_cb(self, msg):
        self.last_pose = msg
        self.try_publish()

    def try_publish(self):
        if self.last_rssi and self.last_pose:
            fused = WifiFused()
            fused.timestamp = self.last_rssi.timestamp
            fused.x = float(self.last_pose.x)
            fused.y = float(self.last_pose.y)
            fused.rssi = self.last_rssi.rssi

            self.pub.publish(fused)
            self.get_logger().info(
                f"FUSED x={fused.x:.2f}, y={fused.y:.2f}, rssi={fused.rssi}"
            )

def main():
    rclpy.init()
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
