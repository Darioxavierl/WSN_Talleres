FROM ubuntu:latest

ARG USERNAME
ENV USERNAME=${USERNAME}

# se instala herramientas
RUN apt update && apt upgrade -y

RUN  apt install -y \
    sudo git iproute2 net-tools iputils-ping can-utils curl x11-apps

RUN apt install -y \
    build-essential 

RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p /home/${USERNAME}/can && \
 chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/can

USER ${USERNAME}

WORKDIR /home/${USERNAME}/can

# Para mantener el contenedor vivo
CMD ["tail", "-f", "/dev/null"]



