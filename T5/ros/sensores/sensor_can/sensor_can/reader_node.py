import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import can


class SensorNode(Node):
    def __init__(self):
        super().__init__("sensor_node")
        self.bus = "can0"
        self.id1=0x123
        self.publisher_ = self.create_publisher(Float32, "temperature", 10)
        self.bus = can.interface.Bus(channel=self.bus, bustype="socketcan")
        self.get_logger().info(f"Escuchando interfaz {self.bus}...")
        self.timer = self.create_timer(0.1, self.read_can)

    def read_can(self):
        msg = self.bus.recv(timeout=0.01)
        if msg and msg.arbitration_id == self.id1 and len(msg.data)>=2:
            tl0 = (msg.data[0]<<8) | msg.data[1]
            if tl0 & 0x8000:
                tl0-=65536
            
            tempC = tl0 /10
            self.get_logger().info(f"Recibido temp: {tempC:.1f} C")
            self.publisher_.publish(Float32(data=tempC))
def main():
    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
