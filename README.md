# Local Pathfinding 

[![CircleCI](https://circleci.com/gh/UBCSailbot/local-pathfinding.svg?style=shield&circle-token=d1bf596ed78d6a5d3744417a589e9ea71128364b)](https://circleci.com/gh/UBCSailbot/local-pathfinding)

The main planning code can be found in the `python` directory.

ROS Topic Inputs:
- `/sensors` - Sensors message containing GPS, wind sensor, accelerometer data, and more. 
- `/AIS` - AISMsg message containing latlon, heading, and speed information about other ships
- `/globalPath` - path message containing a list of latlon waypoints

ROS Topic Outputs:
- `/desired_heading_degrees` - heading message containing the desired heading the autonomous boat should follow (for downstream controller)

![UBC Sailbot Local Pathfinding Diagram July 10 2021 png](https://user-images.githubusercontent.com/26510814/125177488-feb8db80-e190-11eb-8d72-b7960b9380c3.png)

## How to run

Run the installation instructions [here](install/README.md) before running.

### Running Local Pathfinding Main Loop + Mock Inputs + Visualizer 

The easiest way to run the local pathfinding system with ROS is to use multiple terminals. For each terminal, you will need to run `source ~/catkin_ws/devel/setup.bash` before running other commands (you can put this in your `~/.bashrc` file as well to do this automatically). To run the full pathfinding simulation, you can run: `roslaunch local_pathfinding all.launch"`. This runs:

* __The local pathfinding system__, which includes the main_loop, ros_interface, and global_wind nodes. This systems listens to input ROS topics, decides if it needs to recalculate a new local path, and then publishes a desired heading to be used by the downstream controller.

* __The local path visualizer__, which uses `matplotlib` to visualize the boat's position and heading, the local path, the other boats, and the wind speed and direction.

* __The mock inputs__, which are a placeholder for the real inputs.

![UBC Sailbot Local Pathfinding All Launch Aug 11 2021](https://user-images.githubusercontent.com/26510814/129068295-df42f08d-37a3-4b7c-9e50-b219e24ed2b6.png)

#### Visualizing the simulation

The pathfinding tries to avoid large turns, avoid sailing upwind or downwind, and minimize the path length. It prefers straight line paths if possible, but when the wind does not permit, it will generate paths with tacking maneuvers. The cost function to create these tacking paths was developed from a sailboat reactive planning paper, which can be found [here](docs/Tacking_Paper.pdf).

The local path visualizer creates a visualization of the system, which is shown below. The red triangle is the autonomous sailboat, the star is the next global waypoint, the light red circle/wedges are other boats, the dark blue circle/wedges are the projected positions of other boats, and the top left arrow shows the direction of the wind. Note that the local path avoid sailing upwind by sailing at a 45 degree angle with respect to the wind and that the local path only avoids the projected positions of other boats, which incentivizes it to sail behind other boats.

![Figure_1-1](https://user-images.githubusercontent.com/26510814/125178013-89033e80-e195-11eb-95f3-0143269f00fa.png)

You can also open OpenCPN to visualize the path finding progression over the entire map. The OpenCPN set up instructions can be found [here](install/visualisation.md)

![opencpn_visualizer](https://user-images.githubusercontent.com/26510814/125177064-cb288200-e18d-11eb-8ffd-29df13ef60e3.png)

#### Arguments for all.launch

All of the arguments below are optional for `all.launch`. Below is a non-exhaustive list of the arguments.

* `speedup` - Float value that changes the speed at which the simulation is run. Default: `1.0`

* `global_wind_direction_degrees 70` - Float value that sets the global wind direction.

* `global_wind_speed_kmph 5` - Float value that sets the global wind speed.

* `obstacle_type` - String value that changes the obstacle representation of AIS ships. Accepted types are: `"ellipse"`, `"wedge"`,`"circles"`, `"hybrid_circle"`, and `"hybrid_ellipse"`. Default: `"hybrid_circle"`

* `screenshot` - Bool value that sets if screenshots should be taken each time `createPath` is run. Default: `false`.

* `plot_pathfinding_problem` - Bool value that sets if a plot of the pathfinding problem should shown each time `createPath` is run. Default: `false`.

* `random_seed` - Int value that we set the random seed to. Default: `""`, which results in the random seed being set by time.

* `wandb` - Bool value that sets if data should be logged to [wandb](https://wandb.ai/ubcsailbot) for analysis. Need to login with UBC Sailbot credentials to log. Default: `false`.

#### Run a specific saved pathfinding scenario

Instructions to run pathfinding unit tests can be found [here](json/README.md).

#### Interacting with the simulator

During a simulation, you can run:

* `rostopic pub /requestLocalPathUpdate` then press TAB repeatedly to get a default messsage. Then send it to request a local path change, which will only actually change if the new path is lower cost than the current one.

* `rostopic pub /forceLocalPathUpdate` then press TAB repeatedly to get a default messsage. Then send it to force a local path change, which will change the path, regardless if the new path is lower cost than the current one or not.

* `rostopic pub /changeGPS` then press TAB repeatedly to get a default message. Edit the message to get sailbot position and speed that you want. This will change the sailbot position.

* `rosparam set /global_wind_direction_degrees 70` to update the global wind direction.

* `rosparam set /global_wind_speed_kmph 5` to update the global wind speed.

* `rosparam set /plot_pathfinding_problem true` to start plotting the pathfinding problem at each call to `createPath`. Can change `true` to `false` to turn it off.

* `rosparam set /screenshot true` to start screenshotting at each call to `createPath`. Can change `true` to `false` to turn it off.

* `rosparam set /obstacle_type <selected_obstacle_type>` (replace <selected_obstacle_type> with the one you want) to change the obstacle representation type. Accepted values are described in the arguments section above.

* To add or remove AIS boats, you can refer to the instructions [here](python/README.md).

#### Seeing more information about a simulation

You can run `rostopic echo /` then press TAB repeatedly to see the available topics for listening. One of the most useful is `rostopic echo /localPathCostBreakdown` to see how the local path cost is being calculated.

### Running tests

To run the tests, navigate to the `catkin_ws` and run `catkin_make run_tests`.

### Lint

To ensure that the codebase stays clean, we will be using [flake8](https://flake8.pycqa.org/en/latest/) to enforce our style guide, which is mostly based off of [pep8](https://www.python.org/dev/peps/pep-0008/). To automate most of this process, you can use [autopep8](https://github.com/hhatto/autopep8), which is a tool that automatically resolves most style issues. Example usage is shown below. Our team has agreed upon a 120 character line limit.

`pip install flake8`

`pip install --upgrade autopep8`

`autopep8 --in-place --aggressive --max-line-length 120 --aggressive <path_to_file>`

`flake8 --statistics --max-line-length 120 <path_to_file>`

### Continuous Integration

To ensure that the code continues to work as we make changes, we have setup continuous integration with CircleCI. It pulls the Dockerhub image for ROS and OMPL, sets up the workspace, and runs the lint and tests. Click [here](.circleci/config.yml) to see how this is done.
