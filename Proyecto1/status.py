import docker
import time
import csv

container_name = "ros-pj".strip()
client = docker.from_env()

print(f"Monitoreando contenedor: {container_name}")

filename = "docker_metrics.csv"
fields = ["timestamp", "cpu_percent", "memory_mb", "net_rx_mb", "net_tx_mb", "blk_read_mb", "blk_write_mb"]

with open(filename, mode="w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(fields)

    while True:
        try:
            container = client.containers.get(container_name)
            stats = container.stats(stream=False)

            # --- CPU seguro ---
            cpu_delta = stats["cpu_stats"]["cpu_usage"]["total_usage"] - stats["precpu_stats"]["cpu_usage"]["total_usage"]
            system_delta = stats["cpu_stats"]["system_cpu_usage"] - stats["precpu_stats"]["system_cpu_usage"]
            percpu = stats["cpu_stats"]["cpu_usage"].get("percpu_usage", [])
            num_cpus = len(percpu) if percpu else stats["cpu_stats"].get("online_cpus", 1)
            cpu_percent = (cpu_delta / system_delta) * num_cpus * 100.0 if system_delta > 0 else 0.0

            # --- Memoria ---
            mem_usage = stats["memory_stats"]["usage"] / (1024 ** 2)

            # --- Red (seguro) ---
            networks = stats.get("networks", {})
            if networks:
                rx_bytes = sum(net.get("rx_bytes", 0) for net in networks.values()) / (1024 ** 2)
                tx_bytes = sum(net.get("tx_bytes", 0) for net in networks.values()) / (1024 ** 2)
            else:
                rx_bytes, tx_bytes = 0.0, 0.0

            # --- Disco (seguro) ---
            blkio = stats.get("blkio_stats", {}).get("io_service_bytes_recursive", [])
            if blkio:
                read_bytes = sum(x.get("value", 0) for x in blkio if x.get("op") == "Read") / (1024 ** 2)
                write_bytes = sum(x.get("value", 0) for x in blkio if x.get("op") == "Write") / (1024 ** 2)
            else:
                read_bytes, write_bytes = 0.0, 0.0

            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            writer.writerow([ts, cpu_percent, mem_usage, rx_bytes, tx_bytes, read_bytes, write_bytes])
            f.flush()

            print(f"[{ts}] CPU: {cpu_percent:.2f}% | RAM: {mem_usage:.2f} MB | NET RX/TX: {rx_bytes:.2f}/{tx_bytes:.2f} MB")
            time.sleep(1)

        except docker.errors.NotFound:
            print(f"Contenedor {container_name} no encontrado. Reintentando en 5s...")
            time.sleep(5)
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(2)

