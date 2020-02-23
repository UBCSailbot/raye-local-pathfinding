#!/usr/bin/env python
from ompl import util as ou
import rospy
import time
from plotting import plot_path, plot_path_2
from updated_geometric_planner import plan, Obstacle, hasNoCollisions
from cli import parse_obstacle
import math 
from geopy.distance import distance
import geopy.distance
from local_pathfinding.msg import latlon, AISShip
import numpy as np
import matplotlib.pyplot as plt

# Constants
AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM = 3
GLOBAL_WAYPOINT_REACHED_RADIUS_KM = 5
PATH_UPDATE_TIME_LIMIT_SECONDS = 5000

# Constants for bearing and heading
BEARING_NORTH = 0
BEARING_EAST = 90
BEARING_SOUTH = 180
BEARING_WEST = 270

HEADING_EAST = 0
HEADING_NORTH = 90
HEADING_WEST = 180
HEADING_SOUTH = 270

# Constants for boat frame angles
BOAT_RIGHT = 0
BOAT_FORWARD = 90
BOAT_LEFT = 180
BOAT_BACKWARD = 270

def latlonToXY(latlon, referenceLatlon):
    x = distance((referenceLatlon.lat, referenceLatlon.lon), (referenceLatlon.lat, latlon.lon)).kilometers
    y = distance((referenceLatlon.lat, referenceLatlon.lon), (latlon.lat, referenceLatlon.lon)).kilometers
    if referenceLatlon.lon > latlon.lon:
        x = -x
    if referenceLatlon.lat > latlon.lat:
        y = -y
    return [x,y]

def XYToLatlon(xy, referenceLatlon):
    x_distance = geopy.distance.distance(kilometers = xy[0])
    y_distance = geopy.distance.distance(kilometers = xy[1])
    destination = x_distance.destination(point=(referenceLatlon.lat, referenceLatlon.lon), bearing=BEARING_EAST)
    destination = y_distance.destination(point=(destination.latitude, destination.longitude), bearing=BEARING_NORTH)
    return latlon(destination.latitude, destination.longitude)

def createLocalPathSS(state):
    ou.setLogLevel(ou.LOG_WARN)

    # Get setup parameters from state for ompl plan()
    # Convert all latlons to NE in km wrt referenceLatlon
    referenceLatlon = state.position
    start = latlonToXY(state.position, referenceLatlon)
    goal = latlonToXY(state.globalWaypoint, referenceLatlon)
    extraKm = 10   # Extra length to show more in the plot
    dimensions = [min(start[0], goal[0]) - extraKm, min(start[1], goal[1]) - extraKm, max(start[0], goal[0]) + extraKm, max(start[1], goal[1]) + extraKm]
    obstacles = extendObstaclesArray(state.AISData.ships, referenceLatlon)
    globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
    runtime = 5

    # Run the planner multiple times and find the best one
    numRuns = 3
    rospy.loginfo("Running createLocalPathSS. runTime: {}. numRuns: {}. Total time: {} seconds".format(runtime, numRuns, runtime*numRuns))
    solutions = []
    for i in range(numRuns):
        # TODO: Incorporate globalWindSpeed into pathfinding?
        rospy.loginfo("Starting path-planning run number: {}".format(i))
        solution = plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', globalWindDirectionDegrees, dimensions, start, goal, obstacles)
        if solution.haveExactSolutionPath():
            solutions.append(solution)
        else:
            rospy.logwarn("Solution number {} does not have exact solution path".format(i))

    # Check if no solutions found
    if len(solutions) == 0:
        # TODO: Figure out what to do if no solution is found. Eg. try again with high runtime
        rospy.logerr("No solutions found")
        raise Exception("No solutions found")

    # Find best solution, but catch exception when solution doesn't work
    # TODO: Figure out more about why solutions fail to implement better exception handling
    try:
        solution = min(solutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())
    except:
        print("Solution failed!")

    # Set the average distance between waypoints
    localPathLengthKm = solution.getSolutionPath().length()
    numberOfLocalWaypoints = int(localPathLengthKm / AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM)
    solution.getSolutionPath().interpolate(numberOfLocalWaypoints)

    return solution, referenceLatlon

def getLocalPath(localPathSS, referenceLatlon):
    # Convert localPathSS solution path (in km WRT reference) into list of latlons
    localPath = []
    for state in localPathSS.getSolutionPath().getStates():
        xy = (state.getX(), state.getY())
        localPath.append(XYToLatlon(xy, referenceLatlon))
    return localPath

def badPath(state, localPathSS, referenceLatlon, desiredHeadingMsg):
    # If sailing upwind or downwind, isBad
    globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
    if math.fabs(globalWindDirectionDegrees - desiredHeadingMsg.headingDegrees) < 30 or math.fabs(globalWindDirectionDegrees- desiredHeadingMsg.headingDegrees - math.radians(180)) < 30:
        rospy.logwarn("Sailing upwind/downwind. Global Wind direction: {}. Desired Heading: {}".format(globalWindDirectionDegrees, desiredHeading.headingDegrees))
        return True

    # Check if path will hit objects
    obstacles = extendObstaclesArray(state.AISData.ships, referenceLatlon)
    if not hasNoCollisions(localPathSS, obstacles):
        rospy.logwarn("Going to hit obstacle.")
        return True

    return False

def globalWaypointReached(position, globalWaypoint):
    sailbot = (position.lat, position.lon)
    waypt = (globalWaypoint.lat, globalWaypoint.lon)
    dist = distance(sailbot, waypt).kilometers
    rospy.loginfo("Distance to globalWaypoint is {}".format(dist))
    return distance(sailbot, waypt).kilometers < GLOBAL_WAYPOINT_REACHED_RADIUS_KM

def localWaypointReached(position, localPath, localPathIndex, refLatlon):
    positionX, positionY = latlonToXY(position, refLatlon)
    previousWaypoint = localPath[localPathIndex - 1]
    localWaypoint = localPath[localPathIndex]
    previousWaypointX, previousWaypointY = latlonToXY(latlon(previousWaypoint.lat, previousWaypoint.lon), refLatlon)
    localWaypointX, localWaypointY = latlonToXY(latlon(localWaypoint.lat, localWaypoint.lon), refLatlon)
    isStartNorth = localWaypointY < previousWaypointY 
    isStartEast = localWaypointX < previousWaypointX

    if localWaypointX == previousWaypointX:
        if isStartNorth:
            return positionY <= localWaypointY
        else:
            return positionY >= localWaypointY
    if localWaypointY == previousWaypointY:
        if isStartEast:
            return positionX <= localWaypointX
        else:
            return positionX >= localWaypointX
            
    tangentSlope = (localWaypointY - previousWaypointY) / (localWaypointX - previousWaypointX)
    normalSlope = -1/tangentSlope
    
    if localWaypointX > 0:
        b = localWaypointY + normalSlope * -math.fabs(localWaypointX)
    else:
        b = localWaypointY + normalSlope * math.fabs(localWaypointX)
    y = lambda x: normalSlope * x + b
    x = lambda y: (y - b) / normalSlope 

#    plt.xlim(-20, 20)
#    plt.ylim(-20, 20)
#    plt.plot([0], [0], marker = 'o', markersize=10, color="black")
#    plt.plot([positionX], [positionY], marker = 'o', markersize=10, color="blue")
#    plt.plot([previousWaypointX], [previousWaypointY], marker = 'o', markersize=10, color="green")
#    plt.plot([localWaypointX], [localWaypointY], marker="o", markersize=10, color="red")
#    x_plot = np.linspace(-200, 200, 100)
#    plt.plot(x_plot, y(x_plot), '-r')
#    plt.show()
    
    if isStartNorth: 
        if positionY < y(positionX):
            return True
    elif positionY > y(positionX):
        return True

    if isStartEast: 
        if positionX < x(positionY):
            return True
    elif positionX > x(positionY):
        return True

    return False

def timeLimitExceeded(lastTimePathCreated):
    return time.time() - lastTimePathCreated > PATH_UPDATE_TIME_LIMIT_SECONDS

def getLocalWaypointLatLon(localPath, localPathIndex):
    # If local path is empty, return (0, 0)
    if len(localPath) == 0:
        rospy.logwarn("Local path is empty.")
        rospy.logwarn("Setting localWaypoint to be (0, 0).")
        return latlon(0, 0)

    # If index out of range, return last waypoint in path
    if localPathIndex >= len(localPath):
        rospy.logwarn("Local path index is out of range: index = {} len(localPath) = {}".format(localPathIndex, len(localPath)))
        rospy.logwarn("Setting localWaypoint to be the last element of the localPath")
        localPathIndex = len(localPath) - 1
        return localPath[localPathIndex]

    # If index in range, return the correct waypoint
    else:
        return localPath[localPathIndex]

def getDesiredHeading(position, localWaypoint):
    xy = latlonToXY(localWaypoint, position)
    return math.degrees(math.atan2(xy[1], xy[0]))

def extendObstaclesArray(aisArray, referenceLatLon):
#assuming speed in km/h
    obstacles = []
    timeToLoc = 1  # (hours assuming km/h) change this value when deciding how much to extend obstacles
    radius = 0.2    # also change this to account for the width of the obstacle 
    spacing = 0.2 # can be changed to change width between obstacles

    for aisData in aisArray:
        aisX, aisY = latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatLon)
        if aisData.headingDegrees == 90 or aisData.headingDegrees == 270:
            if aisData.headingDegrees == 90:
                endY = aisY + aisData.speedKmph * timeToLoc
                yRange = np.arange(aisY, endY, spacing)
            if aisData.headingDegrees == 270:
                endY = aisY - aisData.speedKmph * timeToLoc
                yRange = np.arange(endY, aisY, spacing)
            for y in yRange:
                obstacles.append(Obstacle(aisX, y, radius))
        else:
            isHeadingWest = aisData.headingDegrees < 270 and aisData.headingDegrees > 90
            slope = math.tan(math.radians(aisData.headingDegrees))
            dx = spacing / math.sqrt(1 + slope**2)

            if aisX > 0:
                b = aisY + slope * -math.fabs(aisX)
            else:
                b = aisY + slope * math.fabs(aisX)
            xDistTravelled =  math.fabs(aisData.speedKmph * timeToLoc * math.cos(math.radians(aisData.headingDegrees)))
            y = lambda x: slope * x + b 
            if isHeadingWest:
                endX = aisX - xDistTravelled
                xRange = np.arange(endX, aisX, dx)
            else:
                endX = aisX + xDistTravelled
                xRange = np.arange(aisX, endX, dx)
            for x in xRange:
                obstacles.append(Obstacle(x, y(x), radius))
    return obstacles

def measuredWindToGlobalWind(measuredWindSpeed, measuredWindDirectionDegrees, boatSpeed, headingDegrees):
    # Calculate wind speed in boat frame. X is right. Y is forward.
    measuredWindSpeedXBoatFrame = measuredWindSpeed * math.cos(math.radians(measuredWindDirectionDegrees))
    measuredWindSpeedYBoatFrame = measuredWindSpeed * math.sin(math.radians(measuredWindDirectionDegrees))

    # Assume boat is moving entirely in heading direction, so all boat speed is in boat frame Y direction
    trueWindSpeedXBoatFrame = measuredWindSpeedXBoatFrame
    trueWindSpeedYBoatFrame = measuredWindSpeedYBoatFrame + boatSpeed

    # Calculate wind speed in global frame. X is EAST. Y is NORTH.
    trueWindSpeedXGlobalFrame = trueWindSpeedXBoatFrame * math.sin(math.radians(headingDegrees)) + trueWindSpeedYBoatFrame * math.cos(math.radians(headingDegrees))
    trueWindSpeedYGlobalFrame = trueWindSpeedYBoatFrame * math.sin(math.radians(headingDegrees)) - trueWindSpeedXBoatFrame * math.cos(math.radians(headingDegrees))

    # Calculate global wind speed and direction
    globalWindSpeed = (trueWindSpeedXGlobalFrame**2 + trueWindSpeedYGlobalFrame**2)**0.5
    globalWindDirectionDegrees = math.degrees(math.atan2(trueWindSpeedYGlobalFrame, trueWindSpeedXGlobalFrame))

    return globalWindSpeed, globalWindDirectionDegrees

def globalWindToMeasuredWind(globalWindSpeed, globalWindDirectionDegrees, boatSpeed, headingDegrees):
    # Calculate the measuredWindSpeed in the global frame
    measuredWindSpeedXGlobalFrame = globalWindSpeed * math.cos(math.radians(globalWindDirectionDegrees)) - boatSpeed * math.cos(math.radians(headingDegrees))
    measuredWindSpeedYGlobalFrame = globalWindSpeed * math.sin(math.radians(globalWindDirectionDegrees)) - boatSpeed * math.sin(math.radians(headingDegrees))

    # Calculated the measuredWindSpeed in the boat frame
    measuredWindSpeedXBoatFrame = measuredWindSpeedXGlobalFrame * math.sin(math.radians(headingDegrees)) - measuredWindSpeedYGlobalFrame * math.cos(math.radians(headingDegrees))
    measuredWindSpeedYBoatFrame = measuredWindSpeedXGlobalFrame * math.cos(math.radians(headingDegrees)) + measuredWindSpeedYGlobalFrame * math.sin(math.radians(headingDegrees))

    measuredWindDirectionDegrees = math.degrees(math.atan2(measuredWindSpeedYBoatFrame, measuredWindSpeedXBoatFrame))
    measuredWindSpeed = (measuredWindSpeedYBoatFrame**2 + measuredWindSpeedXBoatFrame**2)**0.5

    return measuredWindSpeed, measuredWindDirectionDegrees

def headingToBearingDegrees(headingDegrees):
    # Heading is defined using cartesian coordinates. 0 degrees is East. 90 degrees in North. 270 degrees is South.
    # Bearing is defined differently. 0 degrees is North. 90 degrees is East. 180 degrees is South.
    # Heading = -Bearing + 90
    return -headingDegrees + 90
