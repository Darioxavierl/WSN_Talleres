'''
Nodo que sirve como puente para enviar los comandos con el dron, abre un socket para los comandos tipo Bind,
asi que se vuelve en el unico que mandara comandos al dron. 
Este nodo se comunica con los demas enviando la informacion que requieren.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
import time
import random 
from dron.tello.tello_lib import Tello


class comanderNode(Node):
    def __init__(self):
        super().__init__("comander_node")
        
        # Inicia la instancia de la libreria del dron 
        self.tello = Tello(ssid="TELLO-636EBD", timeout=3)

        # Genera el timer que verifica la conexion cada 5 segundos
        self.create_timer(5.0, self.timer_callback)

        # Crea el topico donde se publica el estado de la conexion
        self.publisher_ = self.create_publisher(Float32, "Conexion", 5)

        # Crea un servicio tipo Trigger que permite a cualquier nodo iniciar la transmision de video
        self.create_service(Trigger, "start_video", self.start_video_callback)
        
        # Crea un servicio tipo Trigger que permite detener la transmision de video
        self.create_service(Trigger, "stop_video", self.stop_video_callback)

        # Crea un servicio tipo Trigger para aterrizaje forzoso
        self.create_service(Trigger, "aterriza", self.emergency_land_callback)
        
        # Crea servicios para ejecutar secuencias
        self.srv = self.create_service(Trigger, 'tello_sequence', self.sequence_callback)
        self.srv2 = self.create_service(Trigger, 'tello_sequence2', self.sequence_callback2)

        # Suscripción al tiempo de espera entre comandos
        self.timepo_sub = self.create_subscription(Float32, 'Tiempo', self.tiempo_listener_callback, 10)
        self.tiempo_espera = 0
        
        # Bandera para aterrizaje prioritario
        self.force_land = False

        # Inicia el puerto de comandos
        self.tello._init_command_socket()

    def tiempo_listener_callback(self, msg):
        '''Obtiene el tiempo de espera para la secuencia de vuelo.'''
        self.tiempo_espera = msg.data

    def timer_callback(self):
        '''
        Verifica la conexion con el dron enviando <<command>> al puerto del dron
        y esperando la respuesta. Publica en el topico <<Conexion>> el estado.
        '''
        self.get_logger().info("[+] Probando conexión con el dron...")
        msg = Float32()
        if self.tello.check_connection(2):
            msg.data = 100.0
        else:
            msg.data = 200.0
        self.publisher_.publish(msg)  

    def emergency_land_callback(self, request, response):
        """
        Callback para enviar un aterrizaje forzoso al dron e interrumpir cualquier secuencia activa.
        """
        self.get_logger().warn("[!] Aterrizaje forzoso solicitado, interrumpiendo secuencia...")
        self.force_land = True  # Señal para detener secuencia

        try:
            #self.tello.send_command("land")
            response.success = True
            response.message = "Aterrizaje forzoso ejecutado."
        except Exception as e:
            response.success = False
            response.message = f"Error al aterrizar: {e}"
        return response

    def start_video_callback(self, request, response):
        """
        Inicia la emisión de video del dron.
        """
        self.get_logger().info("Petición 'start_video' recibida...")
        try:
            self.tello._stream_video(1) 
            response.success = True
            response.message = "Video stream iniciado correctamente."
            self.get_logger().info("[+] 'start_video' ejecutado con éxito.")
        except Exception as e:
            response.success = False
            response.message = f"Error al iniciar video: {e}"
            self.get_logger().error(f"[!] 'start_video' falló: {e}")
        return response

    def stop_video_callback(self, request, response):
        """
        Detiene la emisión de video del dron.
        """
        self.get_logger().info("Petición 'stop_video' recibida...")
        try:
            self.tello._stream_video(0)
            response.success = True
            response.message = "Video stream detenido."
        except Exception as e:
            response.success = False
            response.message = f"Error al detener video: {e}"
        return response

    def check_abort(self):
        """
        Verifica si hay una orden de aterrizaje forzoso activa.
        """
        if self.force_land:
            self.get_logger().warn("[!] Secuencia interrumpida por aterrizaje forzoso.")
            self.tello.send_command("land")

            raise InterruptedError("Secuencia abortada por aterrizaje forzoso.")

    def delay(self, seconds, motivo=""):
        """
        Delay con verificación periódica de aterrizaje.
        """
        self.get_logger().info(f"Esperando {seconds:.1f}s {motivo}...")
        start = time.time()
        while time.time() - start < seconds:
            if self.force_land:
                self.get_logger().warn("[!] Delay interrumpido por aterrizaje forzoso.")
                raise InterruptedError("Delay abortado por aterrizaje forzoso.")
            time.sleep(0.1)  # pausas cortas para permitir interrupciones rápidas

    def execute_sequence(self):
        '''
        Secuencia de vuelo básica con control de interrupción.
        '''
        try:
            self.force_land = False  # Reinicia bandera
            self.get_logger().info("Iniciando secuencia de vuelo...")

            self.check_abort()
            self.get_logger().info("Despegue")
            self.tello.send_command("takeoff")

            self.check_abort()
            stab_time = random.uniform(3.0, 5.0)
            self.delay(stab_time, "(estabilización tras despegue)")

            self.check_abort()
            self.get_logger().info("Avance")
            self.tello.send_command("forward 200")
            self.delay(self.tiempo_espera, "(pausa tras avance)")

            self.check_abort()
            self.get_logger().info("Retroceso")
            self.tello.send_command("back 200")
            self.delay(self.tiempo_espera, "(pausa tras retroceso)")

            self.check_abort()
            self.get_logger().info("Giro en clockwise")
            self.tello.send_command("cw 360")
            self.delay(self.tiempo_espera, "(esperando giro completo)")



            self.check_abort()
            self.get_logger().info("Aterrizando...")
            self.tello.send_command("land")
            self.delay(2.0, "(esperando aterrizaje)")

            self.get_logger().info("Secuencia completada con éxito.")
            return True, "Secuencia ejecutada correctamente."

        except InterruptedError:
            # Secuencia interrumpida por aterrizaje forzoso
            return False, "Secuencia interrumpida por aterrizaje forzoso."
        except Exception as e:
            self.get_logger().error(f"Error durante la secuencia: {e}")
            try:
                self.tello.send_command("land")
            except:
                pass
            return False, f"Error: {str(e)}"
        
    def execute_sequence2(self):
        '''
        Segunda secuencia de vuelo con giro y control de interrupción.
        '''
        try:
            self.force_land = False
            self.get_logger().info("Iniciando secuencia de vuelo 2...")

            self.check_abort()
            self.tello.send_command("takeoff")
            self.delay(3.0)

            self.check_abort()
            self.get_logger().info("Movimiento lateral")
            self.tello.send_command("go 0 -30 0 10")
            self.delay(self.tiempo_espera, "(pausa tras avance)")

            self.check_abort()
            self.get_logger().info("Giro de 360°")
            self.tello.send_command("cw 360")
            self.delay(4.0, "(esperando giro completo)")

            self.check_abort()
            self.get_logger().info("Aterrizando...")
            self.tello.send_command("land")
            self.delay(self.tiempo_espera, "(esperando aterrizaje)")

            self.get_logger().info("Secuencia 2 completada con éxito.")
            return True, "Secuencia 2 ejecutada correctamente."

        except InterruptedError:
            return False, "Secuencia 2 interrumpida por aterrizaje forzoso."
        except Exception as e:
            self.get_logger().error(f"Error durante la secuencia 2: {e}")
            try:
                self.tello.send_command("land")
            except:
                pass
            return False, f"Error: {str(e)}"
        
    def sequence_callback(self, request, response):
        '''
        Callback para lanzar la primera secuencia de vuelo.
        '''
        success, message = self.execute_sequence()
        response.success = success
        response.message = message
        return response

    def sequence_callback2(self, request, response):
        '''
        Callback para lanzar la segunda secuencia de vuelo.
        '''
        success, message = self.execute_sequence2()
        response.success = success
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = comanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
