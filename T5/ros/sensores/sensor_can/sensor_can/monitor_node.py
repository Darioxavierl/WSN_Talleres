import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MonitorNode(Node):
    def __init__(self):
        super().__init__("monitor_node")
        self.subscription = self.create_subscription(Float32, "temperature", self.listener_callback,10)
        

    def listener_callback(self, msg):
        self.get_logger().info(f"Temperatura actual {msg.data:.2f} C")

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()