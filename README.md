# Python planning code 

The main planning code in python can be found in the `python` directory. This code is meant to provide a baseline for the C++ implementation.

Inputs:
- Measured wind from sensors
- Other AIS-enabled ships are considered obstacles
- Heading
- GPS location (with speed)
- Output from global path planner

Outputs:
- Desired heading (for the MPC controller)

## How to run

Be sure to run the installation instructions below before running.

### Test OMPL pathfinding 

To test the OMPL pathfinding code through a simple command line interface, run `python2 python/cli.py`.

### Running Local Pathfinding Main Loop + Mock Inputs + Visualizer 

The easiest way to run the local pathfinding system with ROS is to use multiple terminals. For each terminal, you will need to run `source ~/catkin_ws/devel/setup.bash` before running other commands.

* In a new terminal, run `roslaunch local_pathfinding launch_all_mocks.launch` to setup the mock nodes.

* In a new terminal, run `rosrun local_pathfinding main_loop.py` to run the main loop.

* In a new terminal, run `rosrun local_pathfinding local_path_visualizer.py` to run the local path visualizer. This uses Matplotlib to visualize the system. The coordinates are in km with respect to the next global waypoint.

* You can also open OpenCPN to visualize the path finding progression over the entire map.

## Installation

The installation process consists of installing OMPL with Python bindings, setting up your ROS workspace, and setting up OpenCPN for visualization.

### Installing OMPL

ROS requires Python2, so we will have a custom ompl installation script for this task. This may take 7-12 hours, as the creation of Python bindings takes a very long time. If you encounter errors in this process, try to run the commands in the script individually and identify where you are having difficulties

__Setup Option 1 (Recommended): Installation on the host computer__

Installation on the host computer requires running the commands that are in the Dockerfile directly on your computer. A bash script has been made with the same instructions as the Dockerfile to run this.

* Navigate to the `install` directory: `cd install`

* Run the script as root: `sudo ./ompl_install_instructions.bash`

* Check for errors. If none occur, it should look something like:

```
[ 50%] Built target geometric.h
Scanning dependencies of target update_util_bindings
[ 50%] Creating C++ code for Python module util (see pyplusplus_util.log)
...
still alive after 120.328 seconds!
still alive after 180.415 seconds!
```

If the script stops, copy the output to a text file. You can try running the script line by line instead to find where the error occurs. If you get stuck, you can contact Tyler Lum (tylergwlum@gmail.com) for questions.

__Setup Option 2: Docker__

* Navigate to the `install` directory: `cd install`

* Build the docker image by running `docker build --tag ompl_python_2 .` (can replace ompl_python_2 with the tag name of your choice). This operation takes about 10 hours. It creates a Docker image with all the dependencies to run the software.

* Run a docker container by running `docker run -it --name ompl --rm ompl_python_2`, which creates a container from the image created from the previous step. This brings you into an environment with the desired dependencies.

* Note: By default, the plotting will not working from a docker container because it does not have access to your screen. To fix this, you can run the following command in another terminal: `xhost +local:root` (be sure to run `xhost -local:root` when you are done to be safe!), then use the following command instead of the previous docker run command:

```
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ompl_python_2
```

This will allow you to plot paths.

### Setting up ROS Workspace

1. Install ROS Melodic on a Ubuntu 18.04 (or similar). http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

2. Find the location that you want to create a ROS workspace (eg. `cd ~`)

3. Type the commands `mkdir -p catkin_ws/src` `cd catkin_ws` `catkin_make`.

4. Clone the repository in the src folder: `cd src` `git clone https://github.com/UBCSailbot/local-pathfinding.git`. 

5. Go back to catkin_ws and build and source. `cd ..` `cd ..` `catkin_make` `source devel/setup.zsh`.

### Setting up OpenCPN

[How to setup OpenCPN for visualisation](visualisation.md)

