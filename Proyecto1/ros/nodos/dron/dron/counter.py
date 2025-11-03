import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import os
import time


class VideoCounterNode(Node):
    def __init__(self):
        super().__init__('video_counter_node')

        # --- Variables para almacenar conteos ---
        self.red_count = 0
        self.black_count = 0

        # --- Suscripciones a los tópicos ---
        self.sub_red = self.create_subscription(
            Int32, 'red_objects_count', self.red_callback, 10)
        self.sub_black = self.create_subscription(
            Int32, 'black_objects_count', self.black_callback, 10)

        # --- Timer para actualizar la consola ---
        self.timer = self.create_timer(0.5, self.display_table)  # cada 0.5 s

        self.get_logger().info("Nodo 'video_counter_node' iniciado ✅")

    def red_callback(self, msg):
        """Actualiza el conteo de objetos rojos."""
        self.red_count = msg.data

    def black_callback(self, msg):
        """Actualiza el conteo de objetos negros."""
        self.black_count = msg.data

    def display_table(self):
        """Muestra una tabla en consola con los conteos actuales."""
        os.system('clear')  # limpia la terminal (en Windows usa 'cls')
        print("+-------------------+--------------------+")
        print("| Objetos Rojos     | Objetos Negros     |")
        print("+-------------------+--------------------+")
        print(f"|{self.red_count:^19}|{self.black_count:^20}|")
        print("+-------------------+--------------------+")
        print(f"Última actualización: {time.strftime('%H:%M:%S')}")
        print("(Ctrl+C para salir)")

def main(args=None):
    rclpy.init(args=args)
    node = VideoCounterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando nodo de contador...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
