'''
Nodo que verifica el estado de la bateria para mandar el aterrizaje de emergencia de ser necesario.
Se suscribe a los topicos donde se publica, bateria y altura, de este modo detecta si esta en vuelo y si la
bateria esta por debajo el umbral, y manda a ejecutar el comando de aterrizaje.
Ademas publica una bandera para los demas nodos, en el topico
<<bateria_segura>>
De este modo cualquier nodo que quiera mandar a volar al dron primero se asegura que esta flag se lo permita.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import threading

class SafetyNode(Node):
    def __init__(self):
        super().__init__('failsave_node')

        # Variables para guardar los últimos valores
        self.bateria = 100.0
        self.altura = 0.0
        self.landed = True  # Evita enviar múltiples comandos
        self.umbral = 5

        # Suscripciones
        self.create_subscription(Float32, 'bateria', self.bateria_callback, 10)
        self.create_subscription(Float32, 'altura', self.altura_callback, 10)

        # Cliente de servicio para aterrizar
        self.landing_client = self.create_client(Trigger, 'aterriza')
        while not self.landing_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio land_drone...')

        # Publicador de flag de batería segura
        self.pub_bat_segura = self.create_publisher(Float32, 'bateria_segura', 10)

        # Timer para revisar condición cada 0.5s
        self.create_timer(0.5, self.check_safety)

        # Hilo adicional para modificar el umbral desde consola
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def bateria_callback(self, msg):
        #self.get_logger().info(f"Bateria {self.bateria}")
        self.bateria = msg.data
        #self.publish_bateria_segura()
        flag = Float32()
        #flag.data = True if self.bateria > self.umbral else False
        if self.bateria > self.umbral:
            flag.data = 1.0
        else:
            flag.data = 0.0
        self.pub_bat_segura.publish(flag)
        #self.get_logger().info(f"Batería segura: {flag.data}")
    
    def publish_bateria_segura(self):
        flag = Float32()
        #flag.data = True if self.bateria > self.umbral else False
        if self.bateria > self. umbral:
            flag.data = 1
        else:
            flag.data = 0
        self.pub_bat_segura.publish(flag)
        self.get_logger().debug(f"Batería segura: {flag.data}")

    def altura_callback(self, msg):
        self.altura = msg.data

    def check_safety(self):
        if self.altura > 0 and self.bateria < self.umbral and self.landed:
            self.get_logger().warn(f"Batería baja ({self.bateria}%) y altura {self.altura}cm: ejecutando aterrizaje!")
            self.send_land_command()
            self.landed = False
        elif self.altura == 0:
            # Resetea la bandera cuando ya aterrizó
            self.landed = True

    def send_land_command(self):
        req = Trigger.Request()
        future = self.landing_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f"{future.result().message}")
        else:
            self.get_logger().error(f"{future.result().message}")
    
    def input_loop(self):
        """Hilo que permite cambiar el umbral en tiempo real desde consola."""
        while True:
            try:
                new_value = input("Ingrese nuevo valor de umbral (actual={}): ".format(self.umbral))
                if new_value.strip() == "":
                    continue
                new_val = float(new_value)
                self.umbral = new_val
                self.get_logger().info(f"Nuevo umbral establecido: {self.umbral}")
            except ValueError:
                print("Ingrese un número válido.")
            except EOFError:
                break  # Si el input se cierra, termina el hilo

    


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando nodo de seguridad...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
