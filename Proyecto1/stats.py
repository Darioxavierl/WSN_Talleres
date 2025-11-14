import docker
import time
import csv
from datetime import datetime

container_name = "ros-pj".strip()
client = docker.from_env()

print(f"Monitoreando contenedor: {container_name}")

# ========= GENERAR NOMBRE DE ARCHIVO =========
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = f"metricas/docker_metrics_{container_name}_{timestamp}.csv"

fields = [
    "timestamp",
    "cpu_percent",
    "memory_mb",
    "net_rx_mbit",
    "net_tx_mbit",
    "blk_read_mb",
    "blk_write_mb"
]

with open(filename, mode="w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(fields)

    while True:
        try:
            container = client.containers.get(container_name)
            stats = container.stats(stream=False)

            # --- CPU (correcto y seguro) ---
            cpu_delta = stats["cpu_stats"]["cpu_usage"]["total_usage"] - \
                        stats["precpu_stats"]["cpu_usage"]["total_usage"]

            system_delta = stats["cpu_stats"]["system_cpu_usage"] - \
                           stats["precpu_stats"]["system_cpu_usage"]

            percpu = stats["cpu_stats"]["cpu_usage"].get("percpu_usage", [])
            num_cpus = len(percpu) if percpu else stats["cpu_stats"].get("online_cpus", 1)

            cpu_percent = (cpu_delta / system_delta) * num_cpus * 100.0 if system_delta > 0 else 0.0

            # --- Memoria ---
            mem_usage = stats["memory_stats"]["usage"] / (1024 ** 2)

            # --- Red (en MEGABITS por segundo) ---
            networks = stats.get("networks", {})
            if networks:
                rx_mbit = (sum(n.get("rx_bytes", 0) for n in networks.values()) * 8) / (1024 ** 2)
                tx_mbit = (sum(n.get("tx_bytes", 0) for n in networks.values()) * 8) / (1024 ** 2)
            else:
                rx_mbit, tx_mbit = 0.0, 0.0

            # --- Disco (MB) ---
            blkio = stats.get("blkio_stats", {}).get("io_service_bytes_recursive", [])
            if blkio:
                read_mb = sum(x.get("value", 0) for x in blkio if x.get("op") == "Read") / (1024 ** 2)
                write_mb = sum(x.get("value", 0) for x in blkio if x.get("op") == "Write") / (1024 ** 2)
            else:
                read_mb, write_mb = 0.0, 0.0

            ts = time.strftime("%Y-%m-%d %H:%M:%S")

            writer.writerow([ts, cpu_percent, mem_usage, rx_mbit, tx_mbit, read_mb, write_mb])
            f.flush()

            print(f"[{ts}] CPU: {cpu_percent:.2f}% | RAM: {mem_usage:.2f} MB "
                  f"| NET RX/TX: {rx_mbit:.2f}/{tx_mbit:.2f} Mbit")

            time.sleep(1)

        except docker.errors.NotFound:
            print(f"Contenedor {container_name} no encontrado. Reintentando en 5s...")
            time.sleep(5)

        except Exception as e:
            print(f"Error: {e}")
            time.sleep(2)

