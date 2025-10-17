FROM osrf/ros:jazzy-desktop

ARG DIREC
ARG UID
ARG GID
ARG USERNAME

ENV DIREC=${NAME_DIR}
ENV UID=${UID}
ENV GID=${GID}
ENV USERNAME=${USERNAME}

# se instala herramientas de compilacion
RUN apt update && apt upgrade -y && apt install -y python3-colcon-common-extensions vim

# Se crea grupo y usuario no root
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc

COPY ./run-ros-program.sh /home/${USERNAME}/ros2_ws/run-ros-program.sh

RUN chmod +x /home/${USERNAME}/ros2_ws/run-ros-program.sh

RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/ros2_ws

USER ${USERNAME}

WORKDIR /home/${USERNAME}/ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]


CMD ["bash", "-c", "cd /home/${USERNAME}/ros2_ws/src"]

