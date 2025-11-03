# PROYECTO NUMERO 1
## CONTROL DE DRONE TELLO CON ROS

---

### ELABORADO POR GRUPO 3:
    - **David Montano**
    - **Sebastian Pesantez**
    - **Dario Portilla**

---

## Estructura del proyecto:

**Nombre del proyecto de ROS:** `Dron`

### Dependencias:
Asegúrate de tener instaladas las siguientes dependencias:

    - `av` (librería de manejo de video para Python)

**Instalacion de dependencias:**
```bash
sudo apt install python3-<<paquete>>
```
---
### Directorios
Volumen compartidoy carpeta del proyecto:
./ros/nodos → ~/ros2_ws/src/


### Nodos Disponibles:
    - comander_node
    - telemetry_node
    - video_node
    - mision_node
    - viewer_node
    - failsave_node
    - video_detector_node
    - counter_node
    - checker_node


### Ejecucion de nodos
    1. Primero construir:
```bash
colcon build
```
    2. La imagen cuenta con el script "run-ros-program", al que se le pasa como argumentos, el nombre del proyecto y el nombre del nodo, asi se ejecuta, i.e:
```bash
./run-ros-program dron comander_node
```

**Nota:**
Cada nodo debe ser ejecutado en una terminal nueva, entrando al docker
```bash
docker exec -it ros-pj /bin/bash
```
