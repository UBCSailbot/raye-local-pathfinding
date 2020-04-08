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

### Running Local Pathfinding Main Loop + Mock Inputs + Visualizer 

The easiest way to run the local pathfinding system with ROS is to use multiple terminals. For each terminal, you will need to run `source ~/catkin_ws/devel/setup.bash` before running other commands. To run the full pathfinding simulation, you can run: `roslaunch local_pathfinding all.launch initial_speedup:=10 obstacle_type:="wedge"`. This runs:

* __The local pathfinding main loop__, which reads in published sensor data, decides if it needs to recalculate a new local path, and then publishes a desired heading to the MPC controller.

* __The local path visualizer__, which uses Matplotlib to visualize the boat's position and heading, the local path, the other boats, and the wind speed and direction.

* __The mock inputs__, which are a placeholder for the real inputs from sensors and the controller.

#### Visualizing the simulation

The pathfinding tries to avoid large turns, avoid sailing upwind or downwind, and minimize the path length. It prefers straight line paths if possible, but when the wind does not permit, it will generate paths with tacking maneuvers. The cost function to create these tacking paths was developed from a sailboat reactive planning paper, which can be found [here](docs/Tacking_Paper.pdf).

The path can be viewed with the local path visualizer, which is shown below.

![alt text](images/local_path_visualizer.png?raw=true "Local Path Visualizer")

You can also open OpenCPN to visualize the path finding progression over the entire map. The OpenCPN set up instructions can be found below. 

![alt text](images/opencpn_visualizer.png?raw=true "OpenCPN Visualizer")

#### Arguments for all.launch

All of the arguments below are optional for `all.launch`.

* `initial_speedup` - Float value that changes the speed at which the simulation is run. Default: `1.0`

* `obstacle_type` - String value that changes the obstacle representation of AIS ships. Accepted types are: `"ellipse"`, `"wedge"`, and `"circles"`. Default: `"ellipse"`

* `main_loop_output` - String value that changes where ros log information is output. Accepted values are: `screen` and `log`. Default: `screen`.

* `ais_file` - String value that is the full path to a json file with a list of AIS boats. The simulator will read from this file to create its AIS boats. Examples can be found in the `json` folder.

* `gps_file` - String value that is the full path to a json file with a GPS coordinate. The simulator will read from this file to place the sailbot's initial position.

* `wind_file` - String value that is the full path to a json file with a wind speed and direction. The simulator will read from this file to create its initial wind.

* `goal_file` - String value that is the full path to a json file with a GPS coordinate. The simulator will read from this file to select the goal location the sailbot will travel towards.

#### Run a specific saved pathfinding scenario

Instructions to run pathfinding unit tests can be found [here](json/README.md).

#### Interacting with the simulator

During a simulation, you can run:

* `rostopic pub /requestLocalPathUpdate` then press TAB repeatedly to get a default messsage. Then send it to request a local path change, which will only actually change if the new path is lower cost than the current one.

* `rostopic pub /forceLocalPathUpdate` then press TAB repeatedly to get a default messsage. Then send it to force a local path change, which will change the path, regardless if the new path is lower cost than the current one or not.

* `rostopic pub /new_boat` then press TAB repeatedly to get a default messsage. Edit the message to get the boat in the location, speed and direction you want. This will create a new AIS boat obstacle.

* `rostopic pub /changeGlobalWind` then press TAB repeatedly to get a default messsage. Edit the message to get the wind speed and direction you want. This will change the global wind.

* `rostopic pub /changeGPS` then press TAB repeatedly to get a default messsage. Edit the message to get sailbot position that you want. This will change the sailbot position.

* `rostopic pub /delete_boats` then press TAB repeatedly to get a default messsage. Edit the message to remove the AIS boat with the given ID.

* `rostopic pub /boat_on_path` then press TAB repeatedly to get a default messsage. Edit using one of the following options:

1. addType = latlon will just publish a boat with the specified parameters in addBoat.ship
2. addType = nextWaypoint will publish a boat at the next localWaypoint with the specified ID, heading, and speed in addBoat.ship (lat lon params are ignored)
3. addType = onBoat will publish a boat at the current location of the Sailbot with the specified ID, heading, and speed in addBoat.ship (lat lon params are ignored)
4. addType = index will publish a boat at localPath[addBoat.waypointIndex] with the specified ID, heading, and speed in addBoat.ship (lat lon params are ignored)

#### Seeing more information about a simulation

You can run `rostopic echo /` then press TAB repeatedly to see the available topics for listening. One of the most useful is `rostopic echo /localPathCostBreakdown` to see how the local path cost is being calculated.

### Running tests

To run the tests, navigate to the `catkin_ws` and run `catkin_make run_tests`.

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

