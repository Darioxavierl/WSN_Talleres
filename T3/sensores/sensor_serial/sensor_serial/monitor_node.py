import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32, Float32
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS


token = "RvEhIpsYmMo81N_AA5R0qdGOdZ91vMP4C0QU1exK3WraK-lktQiDNr9syFbWXFJjLLUfQzOoheRicFBPC_OSNg=="
org = "my-org"
bucket = "sensores"

class MonitorNode(Node):
    def __init__(self):
        super().__init__("monitor_node")
        qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)
        self.subscription = self.create_subscription(Float32, "temperature_celcius", self.listener_callback,qos)
        self.client = InfluxDBClient(url="http://30.10.10.51:8086", token=token, org=org)
        try:
            self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
        except Exception as e:
            self.get_logger().info(f"Error DB  {e}")


    def listener_callback(self, msg):
        self.get_logger().info(f"Temperatura actual {msg.data:.2f} C")
        point = Point("temperatura").field("valor", msg.data)
        self.write_api.write(bucket=bucket, record=point)

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()