# Unit Testing 

ais\_file - sets AIS boat positions, speeds, and directions

gps\_file - sets sailbot's initial position latlon

goal\_file - sets the goal latlon (note: this is the final goal, not necessarily the first waypoint of the global path if it is sufficiently far away from the sailbot start

wind\_file - sets the initial wind speed and direction

## How to run a test:

* Source the workspace
* Navigate to a json test directory eg: `cd json/test2_2`
* Run pathfinding with the json files in this directory: `roslaunch local_pathfinding all.launch ais_file:=$(pwd)/myAIS.json gps_file:=$(pwd)/myGPS.json goal_file:=$(pwd)/myGoal.json wind_file:=$(pwd)/myWind.json`


Each test1, test2 has a different AIS boat setup, but each test1\_1, test1\_2 has a different wind condition
