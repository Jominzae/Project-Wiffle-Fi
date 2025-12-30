import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class DummyRLNode(Node):
    def __init__(self):
        super().__init__('dummy_rl_node')
        self.pub = self.create_publisher(Point, '/pose/dummy', 10)
        self.x = 0.0
        self.y = 0.0
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        msg = Point()
        msg.x = self.x
        msg.y = self.y
        msg.z = 0.0
        self.pub.publish(msg)

        self.x += 0.2
        self.y += 0.1

def main():
    rclpy.init()
    node = DummyRLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
