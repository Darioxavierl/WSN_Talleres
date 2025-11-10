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

        # Genera el timer que veridica la conexion cada 5 segundos
        self.create_timer(5.0, self.timer_callback)

        # Crea el topico donde se publica el estado de la conexion
        self.publisher_ = self.create_publisher(Float32, "Conexion", 5)

        # Crea un servicio tipo Trigger que permite a cualquier nodo iniciar la transmision de video
        self.create_service(
            Trigger, 
            "start_video", 
            self.start_video_callback
        )
        
        # Crea un servicio tipo Trigger que permite a cualquier nodo terminar la transmision de video
        self.create_service(
            Trigger, 
            "stop_video", 
            self.stop_video_callback
        )

        # Crea un servicio tipo Trigger que permite a cualquier nodo mandar un aterrizaje al dron
        self.create_service(
            Trigger, 
            "aterriza", 
            self.emergency_land_callback
        )
        

        # Crea un servicio tipo Trigger que permite a cualquier nodo iniciar una secuencia de vuelo
        self.srv = self.create_service(Trigger, 'tello_sequence', self.sequence_callback)
        self.srv2= self.create_service(Trigger, 'tello_sequence2', self.sequence_callback2)

        # Se suscribe al topico que permite mandar el tiempo de espera dentro de la secuencia de vuelo
        self.timepo_sub = self.create_subscription(Float32, 'Tiempo', self.tiempo_listener_callback, 10)
        self.tiempo_espera = 0
        
        # Inicio puerto para mandar comandos
        self.tello._init_command_socket()

    def tiempo_listener_callback(self, msg):
        '''
        Obtiene el tiempo de espera para la secuencia de vuelo del topico suscrito.
        '''
        self.tiempo_espera = msg.data

    def timer_callback(self):
       '''
       Este timer verifica la conexion con el dron enviando <<command>> al puerto del dron
       y esperando la respuesta. Entonces publica en el potico <<Conexion>> el estado.
       
       '''
       self.get_logger().info("[+] Probando conexión con el dron...")
       msg = Float32()
       if self.tello.check_connection(2):
            #self.get_logger().info("Conectado!!")
            msg.data = 100.0
       else:
            #self.get_logger().warn("No conectado.")
            msg.data = 200.0
       self.publisher_.publish(msg)  

    def emergency_land_callback(self, request, response):
        """
        Callback para enviar un aterrizaje forzoso al dron, a partir de la orden recibida por el topico.
        """
        self.get_logger().info("Aterrizaje forzoso...")
        try:
            self.tello.land()
            # Si todo salió bien:
            response.success = True
            response.message = "Aterrizaje forzojo enviado"
        except Exception as e:
            response.success = False
            response.message = "Error enviando aterrizaje"
        return response

    def start_video_callback(self, request, response):
        """
        Callback para el servicio 'start_video', inicia la emision de video del dron a partir de la orden recibida 
        por el topico
        """
        self.get_logger().info("Petición 'start_video' recibida...")
        
        try:
            # Aquí va la lógica real: llamas a tu librería de Tello
            self.tello._stream_video(1) 
            
            # Si todo salió bien:
            response.success = True
            response.message = "Video stream iniciado correctamente."
            self.get_logger().info("[+] 'start_video' ejecutado con éxito.")
            
        except Exception as e:
            # Si algo falló:
            response.success = False
            response.message = f"Error al iniciar video: {e}"
            self.get_logger().error(f"[!] 'start_video' falló: {e}")

        # 3. DEVUELVES LA RESPUESTA
        #    Esto es lo que recibirá el cliente.
        return response

    def stop_video_callback(self, request, response):
        """
        Callback para el servicio 'stop_video', detiene la emision de video del dron a partir de la orden recibida 
        por el topico
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

    def delay(self, seconds, motivo=""):
        """
        Metodo para generar el delay dentro de una secuencia de vuelo.
        """
        self.get_logger().info(f"Esperando {seconds:.1f}s {motivo}...")
        time.sleep(seconds)

    def execute_sequence(self):
        '''
        Metodo que permite realizar una secuencia de vuelo.
        Envia los comandos en orden, esperando unicamente el tiempo de delay entre
        cada comando.
        '''
        try:
            self.get_logger().info("Iniciando secuencia de vuelo...")
            self.get_logger().info("despegue")
            # Despegue
            self.tello.send_command("takeoff")
            #self.delay(4.0)

            #self.tello.send_command("rc 0 0 10 0")  # x=20cm/s, y=0, z=0, yaw=0
            #self.delay(4.0)

            

            # Subir a 50 cm
            #for _ in range(2):
            #    self.tello.send_command("rc 0 0 10 0")  # x=20cm/s, y=0, z=0, yaw=0
            #    self.delay(1.0)
            #    self.tello.send_command("rc 0 0 0 0")
            #    self.delay(5)

            self.get_logger().info("estabilizacion")
            # Espera estabilización
            stab_time = random.uniform(3.0, 5.0)
            self.delay(stab_time, "(estabilización tras despegue)")
            self.get_logger().info("avance")
            # Avanzar 50 cm
            # Mueve hacia adelante suavemente durante 1 segundo
            self.tello.send_command("forward 200")  # x=20cm/s, y=0, z=0, yaw=0
            #self.delay(1.0)
            #self.tello.send_command("rc 0 0 0 0")
            self.delay(self.tiempo_espera, "(pausa tras avance)")
            # Avanzar 50 cm
            self.tello.send_command("back 200")
            self.delay(1.0)            # mueve durante 1 segundo
            #self.tello.send_command("rc 0 0 0 0")  # detener 
            self.delay(self.tiempo_espera, "(pausa tras avance)")

            # Aterrizaje
            self.tello.send_command("land")
            self.delay(2.0, "(esperando aterrizaje)")

            self.get_logger().info("Secuencia completada con éxito.")

            return True, "Secuencia ejecutada correctamente."

        except Exception as e:
            self.get_logger().error(f"Error durante la secuencia: {e}")
            self.tello.send_command("land")
            return False, f"Error: {str(e)}"
        
    def execute_sequence3(self):
        '''
        Metodo que permite realizar una secuencia de vuelo.
        Envia los comandos en orden, esperando unicamente el tiempo de delay entre
        cada comando.
        '''
        try:
            self.get_logger().info("Iniciando secuencia de vuelo...")

            self.tello.send_command("takeoff")
            self.delay(3.0)

            # Avanzar 50 cm
            self.tello.send_command("go 0 -3 0 10")
            self.delay(self.tiempo_espera, "(pausa tras avance)")

            # Aterrizaje
            self.tello.send_command("land")
            self.delay(2.0, "(esperando aterrizaje)")

            self.get_logger().info("Secuencia completada con éxito.")
            return True, "Secuencia ejecutada correctamente."

        except Exception as e:
            self.get_logger().error(f"Error durante la secuencia: {e}")
            self.tello.send_command("land")
            return False, f"Error: {str(e)}"
        
    def sequence_callback(self, request, response):
        '''
        Callback que permite lanzar la secuencia de vuelo a partir de la activacion mediante el servicio tipo triger.
        Espera que se terminen de enviar los comando y responde al servicio.
        '''
        success, message = self.execute_sequence()
        response.success = success
        response.message = message
        return response
    def sequence_callback2(self, request, response):
        '''
        Callback que permite lanzar la secuencia de vuelo a partir de la activacion mediante el servicio tipo triger.
        Espera que se terminen de enviar los comando y responde al servicio.
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
