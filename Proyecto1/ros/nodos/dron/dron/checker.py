import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ConexionMonitor(Node):
    def __init__(self):
        super().__init__("checker_node")
        self.subscription = self.create_subscription(
            Float32,
            "Conexion",
            self.listener_callback,
            10
        )
        self.subscription  # evita advertencia de variable sin uso
        self.last_state = None  # para evitar repetir el mismo mensaje muchas veces

    def listener_callback(self, msg):
        estado = msg.data
        if estado == 100.0 and self.last_state != "conectado":
            self.get_logger().info("Dron conectado correctamente.")
            self.last_state = "conectado"
        elif estado == 200.0 and self.last_state != "no_conectado":
            self.get_logger().warn("Dron no conectado.")
            self.last_state = "no_conectado"
        else:
            # No imprimir nada si el estado no cambió
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ConexionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
