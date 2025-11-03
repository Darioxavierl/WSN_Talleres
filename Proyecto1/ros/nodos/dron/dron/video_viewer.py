'''
Nodo que se suscribe al topico de los frames enviados, y los reproduce.
'''


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class VideoViewerNode(Node):
    def __init__(self):
        super().__init__('video_viewer_node')
        self.bridge = CvBridge()

        # Suscripción al topic donde tu nodo Tello publica frames
        self.subscription = self.create_subscription(
            Image,
            'tello_frames',
            self.frame_callback,
            10
        )

        self.subscription  # Evita warning
        self.window_created = False
        self.last_frame_time = time.time()
        self.frame_timeout = 2.0  # segundos sin frame antes de mostrar negro
        self.current_frame = None

        # Timer para refrescar la ventana (incluso si no hay frames)
        self.timer = self.create_timer(0.05, self.update_window)

    def frame_callback(self, msg):
        try:
            # Convertir de ROS Image a OpenCV
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_frame_time = time.time()

            if not self.window_created:
                cv2.namedWindow("Tello Viewer", cv2.WINDOW_NORMAL)
                self.window_created = True

        except Exception as e:
            self.get_logger().error(f"[!] Error procesando frame: {e}")

    def update_window(self):
        # Si no hay ventana, crearla con fondo negro
        if not self.window_created:
            cv2.namedWindow("Tello Viewer", cv2.WINDOW_NORMAL)
            self.window_created = True

        # Si han pasado más de X segundos sin frame → mostrar negro
        if time.time() - self.last_frame_time > self.frame_timeout or self.current_frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "Sin señal de video", (150, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            frame = self.current_frame

        cv2.imshow("Tello Viewer", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VideoViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando nodo visor de video...")
    finally:
        if node.window_created:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
