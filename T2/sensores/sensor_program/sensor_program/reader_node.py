import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReaderNode(Node):
    def __init__(self):
        super().__init__("reader_node")
        self.subscription = self.create_subscription(String, "Sensor_data", self.listener_callback,10)
        self.subscription
    def listener_callback(self, msg):
        self.get_logger().info(f"Recibido: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()