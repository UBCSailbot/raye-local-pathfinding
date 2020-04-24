# Python planning code 

[![UBCSailbot](https://circleci.com/gh/UBCSailbot/local-pathfinding.svg?style=svg&circle-token=	d1bf596ed78d6a5d3744417a589e9ea71128364b)](https://circleci.com/gh/UBCSailbot/local-pathfinding)

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

Be sure to run the installation instruction [here](install/README.md) before running.

### Running Local Pathfinding Main Loop + Mock Inputs + Visualizer 

The easiest way to run the local pathfinding system with ROS is to use multiple terminals. For each terminal, you will need to run `source ~/catkin_ws/devel/setup.bash` before running other commands. To run the full pathfinding simulation, you can run: `roslaunch local_pathfinding all.launch initial_speedup:=10 obstacle_type:="wedge"`. This runs:

* __The local pathfinding main loop__, which reads in published sensor data, decides if it needs to recalculate a new local path, and then publishes a desired heading to the MPC controller.

* __The local path visualizer__, which uses Matplotlib to visualize the boat's position and heading, the local path, the other boats, and the wind speed and direction.

* __The mock inputs__, which are a placeholder for the real inputs from sensors and the controller.

#### Visualizing the simulation

The pathfinding tries to avoid large turns, avoid sailing upwind or downwind, and minimize the path length. It prefers straight line paths if possible, but when the wind does not permit, it will generate paths with tacking maneuvers. The cost function to create these tacking paths was developed from a sailboat reactive planning paper, which can be found [here](docs/Tacking_Paper.pdf).

The path can be viewed with the local path visualizer, which is shown below.

![alt text](images/local_path_visualizer.png?raw=true "Local Path Visualizer")

You can also open OpenCPN to visualize the path finding progression over the entire map. The OpenCPN set up instructions can be found [here](install/visualisation.md)

![alt text](images/opencpn_visualizer.png?raw=true "OpenCPN Visualizer")

#### Arguments for all.launch

All of the arguments below are optional for `all.launch`.

* `initial_speedup` - Float value that changes the speed at which the simulation is run. Default: `1.0`

* `obstacle_type` - String value that changes the obstacle representation of AIS ships. Accepted types are: `"ellipse"`, `"wedge"`,`"circles"`, `"hybrid_circle"`, and `"hybrid_ellipse"`. Default: `"hybrid_circle"`

* `main_loop_output` - String value that changes where ros log information is output. Accepted values are: `screen` and `log`. Default: `screen`.

* `ais_file` - String value that is the full path to a json file with a list of AIS boats. The simulator will read from this file to create its AIS boats. Examples can be found in the `json` folder.

* `gps_file` - String value that is the full path to a json file with a GPS coordinate. The simulator will read from this file to place the sailbot's initial position.

* `wind_file` - String value that is the full path to a json file with a wind speed and direction. The simulator will read from this file to create its initial wind.

* `goal_file` - String value that is the full path to a json file with a GPS coordinate. The simulator will read from this file to select the goal location the sailbot will travel towards.

* `screenshot` - Bool value that sets if screenshots should be taken each time `createPath` is run. Default: `false`.

* `plot_pathfinding_problem` - Bool value that sets if a plot of the pathfinding problem should shown each time `createPath` is run. Default: `false`.

#### Run a specific saved pathfinding scenario

Instructions to run pathfinding unit tests can be found [here](json/README.md).

#### Interacting with the simulator

During a simulation, you can run:

* `rostopic pub /requestLocalPathUpdate` then press TAB repeatedly to get a default messsage. Then send it to request a local path change, which will only actually change if the new path is lower cost than the current one.

* `rostopic pub /forceLocalPathUpdate` then press TAB repeatedly to get a default messsage. Then send it to force a local path change, which will change the path, regardless if the new path is lower cost than the current one or not.

* `rostopic pub /changeGlobalWind` then press TAB repeatedly to get a default messsage. Edit the message to get the wind speed and direction you want. This will change the global wind.

* `rostopic pub /changeGPS` then press TAB repeatedly to get a default messsage. Edit the message to get sailbot position that you want. This will change the sailbot position.

* `rosparam set /plot_pathfinding_problem true` to start plotting the pathfinding problem at each call to `createPath`. Can change `true` to `false` to turn it off.

* `rosparam set /screenshot true` to start screenshotting at each call to `createPath`. Can change `true` to `false` to turn it off.

* `rosparam set /obstacle_type <selected_obstacle_type>` (replace <selected_obstacle_type> with the one you want) to change the obstacle representation type. Accepted values are described in the arguments section above.


* To add or remove AIS boats, you can refer to the instructions [here](python/README.md).

#### Seeing more information about a simulation

You can run `rostopic echo /` then press TAB repeatedly to see the available topics for listening. One of the most useful is `rostopic echo /localPathCostBreakdown` to see how the local path cost is being calculated.

### Running tests

To run the tests, navigate to the `catkin_ws` and run `catkin_make run_tests`.

### Lint

To ensure that the codebase stays clean, we will be using [flake8](https://flake8.pycqa.org/en/latest/) to enforce our style guide, which is mostly based off of [pep8](https://www.python.org/dev/peps/pep-0008/). To automate most of this process, you can use [autopep8](https://github.com/hhatto/autopep8), which is a tool that automatically resolves most style issues. Example usage is shown below. Our team has agreed upon a 120 character line limit. Note that some style choices are still under discussion.

`pip install flake8`

`pip install --upgrade autopep8`

`autopep8 --in-place --aggressive --max-line-length 120 --aggressive <path_to_file>`

`flake8 --statistics --max-line-length 120 <path_to_file>`

### Continuous Integration

To ensure that the code continues to work as we make changes, we have setup continuous integration with CircleCI. It pulls the Dockerhub image for ROS and OMPL, sets up the workspace, and runs the lint and tests. View `.circleci` to see how this is done.
