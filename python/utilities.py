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
    # Convert all latlons to NE in km wrt boat position
    start = [0, 0]
    goal = latlonToXY(state.globalWaypoint, state.position)
    extra = 10   # Extra length to show more in the plot
    dimensions = [min(start[0], goal[0]) - extra, min(start[1], goal[1]) - extra, max(start[0], goal[0]) + extra, max(start[1], goal[1]) + extra]
    obstacles = extendObstaclesArray(state.AISData.ships, state.position)
    measuredWindDirectionDegrees = state.measuredWindDirectionDegrees
    # TODO: FIX MEASURED TO GLOBAL WIND
    runtime = 5

    # Run the planner multiple times and find the best one
    numRuns = 1
    rospy.loginfo("Running createLocalPathSS. runTime: {}. numRuns: {}. Total time: {} seconds".format(runtime, numRuns, runtime*numRuns))
    solutions = []
    for i in range(numRuns):
        rospy.loginfo("Starting run {}".format(i))
        solutions.append(plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', measuredWindDirectionDegrees, dimensions, start, goal, obstacles))

    # Find best solution, but catch exception when solution doesn't work
    # TODO: Figure out more about why solutions fail to implement better exception handling
    try:
        solution = min(solutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())
    except:
        print("Solution did not work")
        print("solutions[0] = {}".format(solutions[0]))

    # Set the average distance between waypoints
    lengthKm = solution.getSolutionPath().length()
    desiredWaypointDistanceKm = 50
    numberOfWaypoints = int(lengthKm / desiredWaypointDistanceKm)
    solution.getSolutionPath().interpolate(numberOfWaypoints)

    return solution, state.position

def getLocalPath(localPathSS, referenceLatlon):
    # Convert localPathSS solution path (in km WRT reference) into list of latlons
    localPath = []
    for state in localPathSS.getSolutionPath().getStates():
        xy = (state.getX(), state.getY())
        localPath.append(XYToLatlon(xy, referenceLatlon))
    return localPath

def badPath(state, localPathSS, referenceLatlon, desiredHeadingMsg):
    # If sailing upwind or downwind, isBad
    # TODO: FIX MEASURED TO GLOBAL WIND
    if math.fabs(state.measuredWindDirectionDegrees - desiredHeadingMsg.headingDegrees) < math.radians(30) or math.fabs(state.measuredWindDirectionDegrees - desiredHeadingMsg.headingDegrees - math.radians(180)) < math.radians(30):
        rospy.logwarn("Sailing upwind/downwind. Wind direction: {}. Desired Heading: {}".format(state.measuredWindDirectionDegrees, desiredHeading.headingDegrees))
        return True

    # Check if path will hit objects
    obstacles = extendObstaclesArray(state.AISData.ships, state.position)
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

def measuredWindToGlobalWind(measuredWindSpeed, measuredWindDirectionDegrees, boatSpeed, boatDirectionDegrees):
    # TODO: Figure out how this works
    return measuredWindSpeed, measuredWindDirectionDegrees
