#!/usr/bin/env bash
set -e

#
# This script takes care of installing all the dependencies required to
#  build the code. Read the comments below for more
#  specific details on what it does.
#
# REQUIREMENTS:
#  - This script only works on Debian-based flavors of Linux, e.g. Ubuntu.
#  - It's assumed that you're starting from a freshly cloned copy of
#     the repository.
#  - A working internet connection is needed to install any missing
# 	  packages as well as the git submodules.
#
#  If you're having problems cloning git repos, check that ssh-agent
#   has your private key loaded. For reference, see e.g.
#   https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/
#

# Clone submodules
git submodule update --init --recursive

# Update package manager indices to latest available.
sudo apt-get update

# Install build tools
sudo apt-get install build-essential cppcheck cmake clang -y

# Install required libraries
sudo apt-get install libeigen3-dev libompl-dev libboost-dev libboost-program-options-dev -y

INSTALL_DEPS_DIRECTORY=${BASH_SOURCE%/*}

LIB_DIRECTORY=${INSTALL_DEPS_DIRECTORY}/../lib

${LIB_DIRECTORY}/protofiles/scripts/install_deps_debian.sh --python
