
FROM ubuntu:latest

RUN apt update && apt upgrade -y && apt-get install iproute2 net-tools iputils-ping -y

RUN apt-get install -y bash vim python3

WORKDIR /

COPY ./contenido /app

CMD ["bash","-c", "echo 'Hello from Ubuntu in Docker Compose!' && sleep infinity"] 

