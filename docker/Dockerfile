# Build from root directory with:
# sudo docker build -t ros2_px4 -f docker/Dockerfile .
FROM ros:galactic-ros-core

LABEL maintainer="George Stavrinos <gstavrinos@iit.demokritos.gr>"

RUN apt-get update && \
      apt-get -y install sudo

RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker
CMD /bin/bash

COPY install.sh /home/docker/install.sh

RUN export DEBIAN_FRONTEND=noninteractive &&\
sudo chmod +x ~/install.sh &&\
~/install.sh
