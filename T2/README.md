# Taller 2 - Captura de tráfico ROS2 con Docker

Este taller consiste en:

1. Configurar un entorno ROS2 dentro de un contenedor Docker.
2. Levanta dos nodos, un sensor que genera un topico con valores simulados de temperatura. Y otro que se suscribe e imprime los datos que recibe. 
3. Agrega un nodo extra que se suscribe y realiza graficos de los datos recibidos cada 5 segundos.
4. Capturar el tráfico de los nodos ROS2 usando un contenedor Netshoot.
5. Guardar y analizar los paquetes `.pcap` generados.


**Archivos importantes:**
- 'compose.yml' : Archivo YAML con las configuraciones de los contenedores a usarse
- 'Dockerfile' : Archivo con la configuracion para la imagen modificada que construyo
- 'run-ros-program.sh' : Bash script que permite ejecutar un nodo, pasando como atributos el nombre del proyecto y el nombre del nodo 
- 'sensores/src' : Volumen compartido donde se accede a  el proyecto de ROS.
- 'sensores/data' : Volumen compartido donde se accede a las graficas del noso de ROS.
- `netshoot/` : Capturas de tráfico `.pcap`.

