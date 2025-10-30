import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray



import can


class SensorNode(Node):
    def __init__(self):
        super().__init__("sensor_node")
        self.bus = "can0"
        self.id1=0x100
        self.id2=0x120
        self.publisher_ = self.create_publisher(Float32MultiArray, "temperature2", 10)
        self.bus = can.interface.Bus(channel=self.bus, bustype="socketcan")
        self.get_logger().info(f"Escuchando interfaz {self.bus}...")
        self.timer = self.create_timer(0.1, self.read_can)

    def read_can(self):
        msg = self.bus.recv(timeout=0.01)
        if msg and len(msg.data)>=2: 
            tempC = None
            tempC2= None

            val = (msg.data[0] << 8) | msg.data[1]
            if val & 0x8000:
                val -= 65536
            temp = val / 10.0

            if msg.arbitration_id == self.id1:
                
                
                tempC = temp
                self.get_logger().info(f"Recibido canID: {self.id1} temp1: {val:.1f}")
                #self.publisher_.publish(Float32(data=tempC))
            elif msg.arbitration_id == self.id2:
                tempC2 = temp
                self.get_logger().info(f"Recibido canID: {self.id2} temp2: {val:.1f}")
                #self.publisher_.publish(Float32(data=tempC2))
            
            msg_out = Float32MultiArray()
            msg_out.data = [
            tempC if tempC is not None else 0.0,
            tempC2 if tempC2 is not None else 0.0
        ]
            self.publisher_.publish(msg_out)


    
def main():
    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
