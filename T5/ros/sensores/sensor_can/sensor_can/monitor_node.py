import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os

class MonitorNode(Node):
    def __init__(self):
        super().__init__('plot_node')
        self.subscription = self.create_subscription(
            Float32MultiArray, 'temperature2', self.listener_callback, 10)

        # Listas para las temperaturas
        self.temp1_data = []
        self.temp2_data = []

        # Detectar si hay entorno gráfico disponible
        if "DISPLAY" not in os.environ or not os.environ["DISPLAY"]:
            self.get_logger().warn("No se detectó entorno gráfico. Usando backend 'Agg' (sin ventana).")
            matplotlib.use("Agg")  # backend sin GUI

        # Configurar gráfico
        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], label='Temp1 (°C)')
        self.line2, = self.ax.plot([], [], label='Temp2 (°C)')
        self.ax.set_xlabel('Muestras')
        self.ax.set_ylabel('Temperatura (°C)')
        self.ax.legend()
        self.ax.grid(True)

        # Timer para refrescar el gráfico cada 0.5 segundos
        self.timer = self.create_timer(0.5, self.update_plot)

        # Mostrar solo si hay entorno gráfico
        self.graphical = matplotlib.get_backend().lower() != "agg"
        if self.graphical:
            plt.show(block=False)

        self.get_logger().info("Nodo de graficado iniciado...")

    def listener_callback(self, msg):
        data = msg.data
        t1 = data[0] if len(data) > 0 else None
        t2 = data[1] if len(data) > 1 else None

        # Guardar solo si son distintos de 0 o None
        if t1 not in [0, None]:
            self.temp1_data.append(t1)
        if t2 not in [0, None]:
            self.temp2_data.append(t2)

        # Mantener solo las últimas 10 muestras
        self.temp1_data = self.temp1_data[-10:]
        self.temp2_data = self.temp2_data[-10:]

        # Guardar la gráfica cada 10 muestras si no hay GUI
        if len(self.temp1_data) == 10 or len(self.temp2_data) == 10:
            if not self.graphical:
                self.update_plot(save=True)
                self.get_logger().info("Gráfico guardado en lsplot_temp.png")

        # Log de los valores recibidos
        if len(data) >= 2:
            self.get_logger().info(f"T1={t1:.2f} °C | T2={t2:.2f} °C")

    def update_plot(self, save=False):
        x_vals1 = np.arange(len(self.temp1_data))
        x_vals2 = np.arange(len(self.temp2_data))

        self.line1.set_data(x_vals1, self.temp1_data)
        self.line2.set_data(x_vals2, self.temp2_data)

        self.ax.relim()
        self.ax.autoscale_view()

        if self.graphical:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        elif save:
            self.fig.canvas.draw()
            self.fig.savefig("src/data/lsplot_temp.png")

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    plt.close()

if __name__ == '__main__':
    main()
