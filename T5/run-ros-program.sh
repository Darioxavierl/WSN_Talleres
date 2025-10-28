#!/bin/bash
# Script para ejecutar un nodo ROS2 desde el workspace

# Verifica que se pasen dos parámetros
if [ "$#" -ne 2 ]; then
    echo "Uso: $0 <paquete> <ejecutable>"
    exit 1
fi

PACKAGE=$1
EXECUTABLE=$2

# Ir al workspace
cd /home/ubuntu/ros2_ws || {
    echo " No se puede acceder a /home/ubuntu/ros2_ws"
    exit 1
}

# Cargar entorno ROS2 del workspace
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo " No se encontró install/setup.bash, ¿ya compilaste con colcon build?"
    exit 1
fi

# Ejecutar el nodo
echo "Ejecutando: ros2 run $PACKAGE $EXECUTABLE"
ros2 run "$PACKAGE" "$EXECUTABLE"
