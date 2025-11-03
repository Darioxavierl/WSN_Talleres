'''
Nodo que permite al usuario empezar la transmision de video del dron.
Permite el inicio mediante control por linea de comandos.
Para el inicio y fin de la transmision se vuelve cliente del servicio tipo Trigger:
<<start_video>>
<<stop_video>>
Este nodo crea una conexion tipo bind al puerto donde se manda el video, obtiene los frames y los publica en el topico.
<<tello_frames>>
'''

import rclpy
from rclpy.node import Node
from dron.tello.tello_lib import Tello
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import threading
import time



class VideoNode(Node):
    def __init__(self):
        super().__init__("video_node_auto")

        # --- Inicialización del dron ---
        self.tello = Tello(ssid="TELLO-636ECF", timeout=3)
        self.Conectado = False

        # --- Variables de control ---
        self._lock = threading.Lock()
        self.streaming = False
        self.publish = True
        self.frame_count = 0
        self.bridge = CvBridge()

        # --- Comunicación ROS ---
        self.subscription = self.create_subscription(Float32, 'Conexion', self.listener_callback, 10)
        self.pub_frames = self.create_publisher(Image, 'tello_frames', 10)
        self.start_video_client = self.create_client(Trigger, "start_video")
        self.stop_video_client = self.create_client(Trigger, "stop_video")

        # Esperar servicios
        while not self.start_video_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio "start_video"...')
        while not self.stop_video_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio "stop_video"...')

        # --- Esperar conexión del dron y luego iniciar stream ---
        threading.Thread(target=self._console_input_loop, daemon=True).start()

    # --- Callback de conexión ---
    def listener_callback(self, msg):
        with self._lock:
            prev_state = self.Conectado
            self.Conectado = (msg.data == 100)

        if self.Conectado and not prev_state:
            self.get_logger().info("→ Dron conectado!")
        elif not self.Conectado and prev_state:
            self.get_logger().warn("→ Dron desconectado!")
            self.stop_video()

    # --- Inicia stream ---
    def start_video(self):
        with self._lock:
            if self.streaming:
                self.get_logger().warn("Stream ya activo.")
                return
            self.streaming = True
            self.frame_count = 0

        try:
            # Configurar callback para cada frame
            self.tello.set_video_callback(self.process_frame)

            # Lanzar decodificador
            self.tello.start_video()
            time.sleep(1)

            # Enviar comando al servidor
            req = Trigger.Request()
            future = self.start_video_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().success:
                self.get_logger().info("[+] Stream activado en dron")
            else:
                self.get_logger().error("[-] Fallo al activar stream en dron")

            self.get_logger().info("[+] Video iniciado correctamente")

        except Exception as e:
            self.get_logger().error(f"[!] Error al iniciar video: {e}")
            self.streaming = False

    # --- Detiene stream ---
    def stop_video(self):
        with self._lock:
            if not self.streaming:
                return
            self.streaming = False

        try:
            req = Trigger.Request()
            self.stop_video_client.call_async(req)
            self.tello.stop_video()
            time.sleep(0.5)
            self.get_logger().info("[+] Video detenido")
        except Exception as e:
            self.get_logger().error(f"[!] Error al detener video: {e}")

    # --- Procesar cada frame ---
    def process_frame(self, frame):
        """Publica cada frame recibido sin mostrarlo"""
        with self._lock:
            if not self.streaming:
                return
            self.frame_count += 1

        if self.publish:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.pub_frames.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"[!] Error publicando frame: {e}")

    # --- Thread de entrada por consola ---
    def _console_input_loop(self):
        while True:
            cmd = input("Comando (start/stop/exit): ").strip().lower()
            if cmd == "start":
                if self.Conectado:
                    self.start_video()
                else:
                    self.get_logger().warn("Dron no conectado")
            elif cmd == "stop":
                self.stop_video()
            elif cmd == "exit":
                self.get_logger().info("Saliendo del control por consola")
                break
            else:
                self.get_logger().info("Comando inválido: start | stop | exit")


def main(args=None):
    rclpy.init(args=args)
    node = VideoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando nodo de video...")
    finally:
        node.stop_video()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
