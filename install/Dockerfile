FROM osrf/ros:melodic-desktop-full-bionic AS builder

###################### OMPL Setup ######################
COPY . /ompl
WORKDIR /ompl
RUN ./ompl_install_instructions.bash
