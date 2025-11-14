# PROYECTO NUMERO 1
## CONTROL DE DRONE TELLO CON ROS

---

### ELABORADO POR GRUPO 3:
    - **David Montano**
    - **Sebastian Pesantez**
    - **Dario Portilla**

---


## Requerimientos
La arquitectura del contenedor está diseñada para soportar interfaces gráficas vía X11, acceso a dispositivos físicos, y comunicación directa con el dron mediante networking en modo host, por lo que el sistema donde se ejecute debe cumplir los siguientes requisitos.

# 1. Sistema Operativo

Se recomienda uno de los siguientes:

- Linux (Ubuntu 20.04/22.04 altamente recomendado). Es el entorno nativo para X11, ROS2 y privilegios de bajo nivel.

- ⚠️ No se garantiza compatibilidad completa con Windows o macOS debido al uso de network_mode: host, acceso a /dev, y forwarding directo de X11.

# 2. Docker y Docker Compose

Instalar:

- Docker Engine ≥ 24.x

- Docker Compose ≥ 2.x

# 3. Servidor gráfico X11

El contenedor usa aplicaciones gráficas basadas en Qt6, por lo que el host debe contar con:

- Servidor X11 funcionando

Permisos para compartir el socket con Docker:
```bash
xhost +local:docker
```
El archivo /tmp/.X11-unix se monta dentro del contenedor, por lo que debe existir en el host.

# 4. Variables de entorno necesarias

El host debe exponer:

- DISPLAY

- USERNAME (ubuntu por defecto)

- SERIAL_PORT
- 
```bash
export DISPLAY=:0
export USERNAME=$USER
export SERIAL_PORT=ttyUSB0
```
# 5. Volúmenes necesarios

El contenedor espera que existan en el host las rutas:
```bash
./ros/nodos/
./ros/data/
```
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
