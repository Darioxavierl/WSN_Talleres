'''
Nodo que permite lanzar una secuencia de vuelo para el dron.
Se vuelve cliente del servicio tipo trigger:
<<tello_sequence>>
Que le permite lanzar la secuencia.
Ademas este nodo se siscribe a los topicos para verificar la conexion con el dron, de altura y de bateria segura,
Para decidir si activar la secuencia de vuelo o no
'''

import rclpy
from rclpy.node import Node
from dron.tello.tello_lib import Tello
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import time
import threading



class MissionNode(Node):
    def __init__(self):
        super().__init__("mision_node")
        # --- Inicialización del dron ---
        self.tello = Tello(ssid="TELLO-636ECF", timeout=3)
        self.Conectado = False
        self.battery_sec = False
        self.altura = 0
        # --- Suscripción ROS ---
        self.subscription = self.create_subscription(Float32, 'Conexion', self.conexion_listener_callback, 10)
        self.sub_altura = self.create_subscription(Float32, 'altura', self.altura_listener_callback, 10)
        self.sub_battery = self.create_subscription(Float32, 'bateria_segura', self.battery_sec_listener_callback, 10)
        self.publisher_ = self.create_publisher(Float32, "Tiempo", 5)

        # --- Servicio ROS ----
        # Cliente para disparar la secuencia
        self.cli = self.create_client(Trigger, 'tello_sequence')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio tello_sequence...')

        self.cli2 = self.create_client(Trigger, 'tello_sequence2')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio tello_sequence.2..')

        # --- Hilo de control interactivo ---
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def altura_listener_callback(self, msg):
        self.altura = msg.data
    def battery_sec_listener_callback(self, msg):
        self.battery_sec = msg.data
        


    def conexion_listener_callback(self, msg):
        data = msg.data
        if data == 100:
            self.Conectado = True
            self.get_logger().info("Dron conectado!")
        elif data == 200:
            self.Conectado = False
            self.get_logger().info("Dron no conectado!")

    def listener_callback(self, msg):
        data = msg.data
        self.altura = data

    def send_trigger(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f"Respuesta: {result.message}")
        else:
            self.get_logger().error("Sin respuesta del servidor.")

    def send_trigger2(self):
        req = Trigger.Request()
        future = self.cli2.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f"Respuesta 2: {result.message}")
        else:
            self.get_logger().error("Sin respuesta del servidor.")

    
    # --- Hilo interactivo ---
    def control_loop(self):
        """Hilo que permite iniciar secuencias manualmente."""
        while rclpy.ok():
            try:
                user_input = input("\nPresiona [ENTER] para iniciar misión, o 'q' para salir: ").strip()
                if user_input.lower() == 'q':
                    self.get_logger().info("Finalizando nodo de misión...")
                    rclpy.shutdown()
                    break

                if not self.Conectado and not self.battery_sec:
                    self.get_logger().warn("Dron no conectado. No se puede iniciar la misión.")
                    continue

                tiempo = input("Ingrese tiempo de pausa tras avance (segundos): ")
                try:
                    tiempo = float(tiempo)
                except ValueError:
                    self.get_logger().warn(" Valor inválido. Usando 3.0s por defecto.")
                    tiempo = 3.0
                msg = Float32()
                msg.data = tiempo
                self.publisher_.publish(msg)

                self.send_trigger()

            except Exception as e:
                self.get_logger().error(f"Error en control_loop: {e}")
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo interrumpido manualmente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()