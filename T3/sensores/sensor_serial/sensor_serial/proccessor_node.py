import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32


class ProccessorNode(Node):
    def __init__(self):
        super().__init__("proccessor_node")
        self.subscription = self.create_subscription(Int32, "sensor_data", self.listener_callback,10)
        self.publisher_ = self.create_publisher(Float32, "temperature_celcius", 10)

    def listener_callback(self, msg):
        raw_value = msg.data
        temperature = (raw_value / 1023) * 100
        temp_msg = Float32()
        temp_msg.data = temperature
        self.get_logger().info(f"Procesado {temperature:.2f} C")
        try:
            self.publisher_.publish(temp_msg)
        except Exception as e:
            self.get_logger().warn(f"Error enviando: {e}") 


def main(args=None):
    rclpy.init(args=args)
    node = ProccessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()