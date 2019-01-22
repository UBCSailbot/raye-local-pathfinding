# Python planning code 

The main planning code in python can be found in `python/planner.py`. This code is meant to provide a baseline for the C++ implementation. 

# Ubuntu OMPL Docker for mac
A docker container for running the Open Motion Planning Library (with GUI) on Mac OS in an Ubuntu environment.

## Install
- Install quartz 2.7.8

## Setup
- `open -a Xquartz`
- Go to Preferences -> security pane -> check `Allow connections from network clients`
- Log out and log back in
- Get IP address: `ip=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')`, add it: `xhost + $ip`

## Run
- Build an image with some name: `docker build -t "<name of build image: tag name>" <path to dockerfile>`
- `docker run -it -e DISPLAY=$ip:0 -v /tmp/.X11-unix:/tmp/.X11-unix <name of build image>`
- From docker bash: `python3 ~/omplapp/gui/ompl_app.py`

