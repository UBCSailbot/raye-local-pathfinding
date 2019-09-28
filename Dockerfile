# Download Ubuntu Bionic
FROM ubuntu:18.04

# Avoid getting stuck at the "Choose your timezone" prompt
ENV DEBIAN_FRONTEND noninteractive

# Install common packages
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    cmake \
    build-essential \
    software-properties-common \
    openssl

# Install dependencies for global and local pathfinding
# TODO: sort them
RUN apt-get update && apt-get install -y \
    libboost-serialization-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libboost-program-options-dev \
    libboost-test-dev \
    libboost-python-dev \
    libeigen3-dev \
    libassimp-dev \
    python3-pyqt5 \
    python3-opengl \
    libode6 \
    libboost-dev \
    freeglut3-dev \
    python3-dev \
    libfcl-dev \
    python3-numpy \
    castxml \
    python3-flask \
    python3-celery \
    python3-pip \
    libompl-dev \
    clang \
    libprotobuf-dev \
    protobuf-compiler \
    libeccodes-dev \
    libeccodes-tools \
    libcurl4-gnutls-dev \
    curl \
    xorg-dev \
    libglew-dev \
    libopenjp2-7-dev \
    libglm-dev \
    cppcheck

RUN pip3 install pyplusplus pygccxml

# Use ROS Melodic
# PS: The key may need updating from time to time
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y ros-melodic-ros-base

# For convenience
RUN apt-get update && apt-get install -y neovim

# Username: sailbot
# Password: raye
RUN useradd -m -p "$(openssl passwd -1 raye)" -G sudo -s /bin/bash sailbot
USER sailbot
WORKDIR /home/sailbot

RUN echo "\n# Source ROS environment setup file" >> .bashrc && \
echo "source /opt/ros/melodic/setup.bash" >> .bashrc

# Go back to default frontend
ENV DEBIAN_FRONTEND dialog

