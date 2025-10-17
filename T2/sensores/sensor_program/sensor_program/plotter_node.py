import rclpy
import matplotlib.pyplot as plt 
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class PlotterNode(Node):
    def __init__(self):
        super().__init__("plotter_node")
        self.subscription = self.create_subscription(String, "Sensor_data", self.listener_callback,10)
        self.subscription
        self.index=0
        self.data = []
        self.i = 0

    def listener_callback(self, msg):
        self.get_logger().info(f"Recibido: {msg.data}")
        if self.index <5:
            self.index+=1
            self.data.append(msg.data.split(" ")[1])
        else:
            self.index = 0
            self.i += 1
            
            fecha = datetime.now()

            self.get_logger().info("Guardando plot")
            plt.figure()
            plt.plot(self.data)
            plt.title("Datos en 5 segundos")
            plt.xlabel("Muestras")
            plt.ylabel("Temperatura C")
            try:
                plt.savefig(f"/home/ubuntu/ros2_ws/src/data/plot{fecha.timestamp()}.png")
                self.get_logger().info("Plot guardado")
                self.data = []
            except Exception as e:
                self.get_logger().info(f"Error en el guardado : {e}")

             

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
