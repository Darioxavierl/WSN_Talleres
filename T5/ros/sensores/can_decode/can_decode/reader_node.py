import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import can


class ReaderNode(Node):
    def __init__(self):
        super().__init__("reader_node")
        self.bus_name = "can0"
        self.id1=0x200

        try:
            self.bus = can.interface.Bus(channel=self.bus_name, bustype="socketcan")
            self.get_logger().info(f"Escuchando en interfaz CAN: {self.bus_name}")
        except can.CanError as e:
            self.get_logger().error(f"Error al abrir bus CAN: {e}")
            return
        
        self.timer = self.create_timer(0.05, self.read_can)
        self.buffer = ""
        self.mensaje_completo = ""

    def read_can(self):
        
        try:
            msg = self.bus.recv(timeout=0.01)
        except can.CanError:
            self.get_logger().warn("Error al leer del bus CAN.")
            return
        
        if msg and msg.arbitration_id == self.id1:
            for byte in msg.data:
                c = chr(byte)
                if c == '<':  # inicio
                    self.buffer = ""
                elif c == '>':  # fin
                    self.mensaje_completo = self.buffer.strip()
                    print("Mensaje completo recibido:", self.mensaje_completo)
                    self.buffer = ""
                else:
                    self.buffer += c
            
def main():
    rclpy.init()
    node = ReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
