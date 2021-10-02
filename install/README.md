# Installation Instructions

The installation process consists of installing OMPL with Python bindings, installing ROS, setting up your ROS workspace, and setting up OpenCPN for visualization.

### Setting up ROS Workspace

1. Install ROS Melodic on Ubuntu 18.04 (or similar): http://wiki.ros.org/melodic/Installation

2. Find the location that you want to create a ROS workspace (eg. home directory: `cd ~`)

3. Create a ROS workspace: `mkdir -p catkin_ws/src` `cd catkin_ws` `catkin_make`

4. Clone this repository in the src folder: `cd src` `git clone https://github.com/UBCSailbot/local-pathfinding.git`

5. Clone the sailbot-msgs repository in the src folder: `git clone https://github.com/UBCSailbot/sailbot-msg.git`

6. Go back to catkin\_ws and build and source. `cd ..` `catkin_make` `source devel/setup.bash`


### Installing OMPL

ROS requires Python2, so we will have a custom ompl installation script for this task. This may take 7-16 hours, as the creation of Python bindings takes a very long time. If you encounter errors in this process, try to run the commands in the script individually and identify where you are having difficulties

__Setup Option 1 (Recommended): Installation on the host computer__

Installation on the host computer requires running the commands that are in the Dockerfile directly on your computer. A bash script has been made with the same instructions as the Dockerfile to run this.

* Navigate to the `install` directory of this repository: `cd src/local-pathfinding` `cd install`

* Run the script as root: `sudo ./ompl_install_instructions.bash --sudo`

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

__Setup Option 2: Pulling Docker Image From DockerHub__

You can also pull a Docker image from DockerHub by running `docker pull tylerlum/ros_ompl_python_2:11.2020.V2`, which was updated as of November 2020. Then you can run:

```
docker run -it tylerlum/ros_ompl_python_2:11.2020.V2
```

Note: By default, the plotting will not working from a docker container because it does not have access to your screen. To fix this, you can run the following command in another terminal: `xhost +local:root` (be sure to run `xhost -local:root` when you are done to be safe!), then use the following command instead of the previous docker run command:

```
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    tylerlum/ros_ompl_python_2:11.2020.V2
```

__Setup Option 3: Creating Docker Image From Scratch (Not recommended)__

* Navigate to the `install` directory: `cd install`

* Build the docker image by running `docker build --tag ros_ompl_python_2 .` (can replace ros\_ompl\_python\_2 with the tag name of your choice). This operation takes about 10 hours. It creates a Docker image with all the dependencies to run the software.

* Run a docker container by running `docker run -it --name ompl --rm ros_ompl_python_2`, which creates a container from the image created from the previous step. This brings you into an environment with the desired dependencies. To get plotting working, you can run `xhost +local:root` then:

```
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ros_ompl_python_2
```
