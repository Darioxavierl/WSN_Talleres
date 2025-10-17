import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class SensorNode(Node):
    def __init__(self):
        super().__init__("sensor_node")
        self.ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
        self.publisher_ = self.create_publisher(Int32, "sensor_data", 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        line = self.ser.readline().decode("utf-8").strip()
        if line.isdigit():
            value = int(line)
            msg = Int32()
            msg.data = value
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publicando crudo {value}")
        else:
            self.get_logger().warn(f"Dato invalido: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()