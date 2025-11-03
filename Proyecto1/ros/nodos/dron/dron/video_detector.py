'''
Nodo que se suscribe al topico de los frames de video, y hace el analisis y conteo de obejetos.
Luego publica la cantidad en dos topicos.
<<red_objects_count>>
<<black_objects_count>>
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class VideoDetectorNode(Node):
    def __init__(self):
        super().__init__('video_detector_node')
        self.bridge = CvBridge()

        # --- Suscripción al topic del dron ---
        self.subscription = self.create_subscription(
            Image,
            'tello_frames',
            self.frame_callback,
            10
        )

        # --- Publicadores ---
        self.pub_red_count = self.create_publisher(Int32, 'red_objects_count', 10)
        self.pub_black_count = self.create_publisher(Int32, 'black_objects_count', 10)

        # --- Ventana ---
        self.window_name = "Tello Object Detection"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # --- Control de frames ---
        self.last_frame_time = time.time()
        self.frame_timeout = 2.0  # segundos sin recibir frame
        self.current_frame = None

        # --- Timer para actualización de ventana ---
        self.timer = self.create_timer(0.05, self.update_window)

        # --- Colores ---
        self.red_lower1 = np.array([0, 120, 70])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 120, 70])
        self.red_upper2 = np.array([180, 255, 255])
        self.black_lower = np.array([0, 0, 0])
        self.black_upper = np.array([180, 255, 40])

        self.min_area = 500

        # --- Control de logs ---
        self.last_log_time = 0.0
        self.log_interval = 1.0  # segundos

    def frame_callback(self, msg):
        """Procesa cada frame recibido y guarda para el procesamiento."""
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_frame = frame_bgr
            self.last_frame_time = time.time()
        except Exception as e:
            self.get_logger().error(f"[!] Error procesando frame: {e}")

    def update_window(self):
        """Actualiza la ventana de video y detección de colores."""
        if self.current_frame is None or time.time() - self.last_frame_time > self.frame_timeout:
            frame_display = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame_display, "Sin señal de video", (150, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow(self.window_name, frame_display)
            cv2.waitKey(1)
            return

        frame_bgr = self.current_frame.copy()
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # --- Detección de color ---
        mask_red1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask_red2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_black = cv2.inRange(hsv, self.black_lower, self.black_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel)

        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_count, black_count = 0, 0

        for contour in contours_red:
            if cv2.contourArea(contour) > self.min_area:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame_bgr, f"Red {red_count + 1}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                red_count += 1

        for contour in contours_black:
            if cv2.contourArea(contour) > self.min_area:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.putText(frame_bgr, f"Blk {black_count + 1}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                black_count += 1

        # --- Mostrar video ---
        cv2.imshow(self.window_name, frame_bgr)
        cv2.waitKey(1)

        # --- Publicar resultados ---
        self.pub_red_count.publish(Int32(data=red_count))
        self.pub_black_count.publish(Int32(data=black_count))

        # --- Log manual con límite de frecuencia ---
        self.log_throttled(f"Detectados: {red_count} rojos, {black_count} negros")

    def log_throttled(self, message: str):
        """Evita imprimir logs más seguido que log_interval."""
        now = time.time()
        if now - self.last_log_time >= self.log_interval:
            self.get_logger().info(message)
            self.last_log_time = now

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando nodo de detección...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
