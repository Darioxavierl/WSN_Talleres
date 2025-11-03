# PROYECTO NUMERO 1
## CONTROL DE DRONE TELLO CON ROS

### ELABORADO POR GRUPO 3:
    - David Montano
    - Sebastian Pesantez
    - Dario Portilla


## Estructura del proyecto:
Nommbre del proyecto de ROS: Dron
### Dependencias:
Verificar las siguientes dependencias:
    - av
Instalacion:
    - sudo apt install python3-<<paquete>>

### Directorios
Volumen compartidoy carpeta del proyecto:
    - ./ros/nodos:~/ros2_ws/src/

### Nodos:
    - comander_node
    - telemetry_node
    - video_node
    - mision_node
    - viewer_node
    - failsave_node
    - video_detector_node
    - counter_node
### Ejecucucion de nodos
    1. Primero contruir con <<colcon build>>
    2. La imagen cuenta con el script "run-ros-program", al que se le pasa como argumentos, el nombre del proyecto y el nombre del nodo. asi se ejecuta, i.e: <<./run-ros-program dron comander_node>>


