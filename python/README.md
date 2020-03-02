# How to add and delete boats

Ensure all nodes are launched. Use `roslaunch local_pathfinding all.launch`

## Add Boats

Type `rostopic pub /new_boats ...` and press Tab twice to generate a message template and fill in the parameters for the ship you would like to add.

## Delete Boats

Type `rostopic pub /delete_boats` and press Tab twice. Fill in the ID of the boats you want to remove.

## Add boats using other methods

Type `rostopic pub /boat_on_path ...` and press Tab twice.

specify the addType as one of the following options:


1. `addType = latlon` will just publish a boat with the specified parameters in addBoat.ship

2. `addType = nextWaypoint` will publish a boat at the next localWaypoint with the specified ID, heading, and speed in addBoat.ship (lat lon params are ignored)

3. `addType = onBoat` will publish a boat at the current location of the Sailbot with the specified ID, heading, and speed in addBoat.ship (lat lon params are ignored)

4. `addType = index` will publish a boat at localPath[addBoat.waypointIndex] with the specified ID, heading, and speed in addBoat.ship (lat lon params are ignored)



