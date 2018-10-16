#Download base image ubuntu
FROM ubuntu:17.10

# Update Ubuntu Software repository and install dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    python3-dev \
    python3-pip \
    python3-numpy \
    pypy \
    doxygen \
    cmake \
    pkg-config \
    libboost-serialization-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libboost-program-options-dev \
    libboost-test-dev \
    libboost-python-dev \
    castxml \
    libeigen3-dev \
    libassimp-dev \
    libfcl-dev \
    libode-dev \
    python3-pyqt5.qtopengl \
    freeglut3-dev \
    python3-opengl \
    python3-flask \
    python3-celery \
    libccd-dev \
&& apt clean \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install pip dependencies
RUN python3 -m pip install pygccxml pyplusplus PyOpenGL-accelerate numpy matplotlib

# Set the working directory to /root
WORKDIR /root

# Install OMPL - Im using 8 cores
RUN git clone https://github.com/ompl/omplapp.git 
WORKDIR /root/omplapp
RUN git clone https://github.com/ompl/ompl.git
WORKDIR /root/omplapp/build/Release
RUN cmake ../.. -DPYTHON_EXEC=/usr/bin/python3 \
&& make -j 8 update_bindings \
&& make -j 8 \
&& make -j 8 install

WORKDIR /root
