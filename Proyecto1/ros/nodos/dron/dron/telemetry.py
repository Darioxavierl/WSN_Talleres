'''
Nodo que se conecta al puerto donde el dron publica toda su informacion. Hace una conexion tipo bind, por lo que es el unico que puede escuchar el puerto.
Se suscribe al topico que verifica la conexion.
Genera dos topicos que sirven para el control y actuacion de otros nodos (altura y bateria).
Filtra el resto de informacion y muestra en consola una tabla con valores de:
    - bateria
    - altura
    - tiempo de vuelo 
    - presion 
    - temperuta minima
    - temperatura maxima
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

from dron.tello.tello_lib import Tello


class TelemetryNode(Node):
    def __init__(self):
        super().__init__("conector_node")
        self.tello = Tello(ssid="TELLO-636ECF")
        self.subscription = self.create_subscription(Float32, 'Conexion', self.listener_callback, 10)
        
        # Publicadores de telemetría (batería, altura, tiempo)
        self.pub_bateria = self.create_publisher(Float32, 'bateria', 10)
        self.pub_altura = self.create_publisher(Float32, 'altura', 10)
        self.pub_tiempo = self.create_publisher(Float32, 'tiempo', 10)
        
        self.create_timer(1.0, self.timer_Telemtry)
        self.Conectado = False

    def listener_callback(self, msg):
        data = msg.data
        if data == 100:
            self.Conectado = True
            self.get_logger().info("Dron conectado!")
        elif data == 200:
            self.Conectado = False
            self.get_logger().info("Dron no conectado!")

    def timer_Telemtry(self):
        if self.Conectado:
            datos = self.tello._listen()
            if datos != None:
                datos =  datos.split("State: ")[0].split(";")
                # Convertimos la lista ['pitch:-7', 'roll:13', ...] a un diccionario {'pitch': -7, 'roll': 13, ...}
                estado = {}
                for d in datos:
                    if ':' in d:
                        k, v = d.split(':')
                        try:
                            estado[k.strip()] = float(v)
                        except ValueError:
                            estado[k.strip()] = v.strip()

                # --- Formato tipo tabla ---
                headers = ["Batería (%)", "Altura (cm)", "Tiempo Vuelo (s)", "Barómetro (hPa)", "Temp. min", "Temp. max"]
                values = [
                    estado.get('bat', 0),
                    estado.get('h', 0),
                    estado.get('time', 0),
                    estado.get('baro', 0),
                    estado.get('templ', 0),
                    estado.get('temph', 0)
                ]

                table = (
                    "\n┌" + "──────────────┬" * (len(headers) - 1) + "──────────────┐\n"
                    "│ " + " │ ".join(f"{h:^14}" for h in headers) + " │\n"
                    "├" + "──────────────┼" * (len(headers) - 1) + "──────────────┤\n"
                    "│ " + " │ ".join(f"{v:^14.2f}" if isinstance(v, (int, float)) else f"{v:^14}" for v in values) + " │\n"
                    "└" + "──────────────┴" * (len(headers) - 1) + "──────────────┘"
                )

                self.get_logger().info(table)
                
                # --- Publicación de tópicos ROS ---
                msg_bat = Float32()
                msg_bat.data = estado.get('bat', 0)
                self.pub_bateria.publish(msg_bat)

                msg_altura = Float32()
                msg_altura.data = estado.get('h', 0)
                self.pub_altura.publish(msg_altura)

                msg_tiempo = Float32()
                msg_tiempo.data = estado.get('time', 0)
                self.pub_tiempo.publish(msg_tiempo)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
