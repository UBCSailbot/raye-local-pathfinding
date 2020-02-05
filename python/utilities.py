#!/usr/bin/env python
from ompl import util as ou
import rospy
import time
from plotting import plot_path
from updated_geometric_planner import plan, Obstacle, hasNoCollisions
from cli import parse_obstacle
import math 
from geopy.distance import great_circle
from local_pathfinding.msg import latlon, AIS_ship
import numpy as np
import matplotlib.pyplot as plt

def createLocalPathSS(state):
    ou.setLogLevel(ou.LOG_WARN)

    # Get setup parameters from state
    start = [state.position.lon, state.position.lat]
    goal = [state.globalWaypoint.lon, state.globalWaypoint.lat]
    extra = 10   # Extra length to show more in the plot
    dimensions = [min(start[0], goal[0]) - extra, min(start[1], goal[1]) - extra, max(start[0], goal[0]) + extra, max(start[1], goal[1]) + extra]
    obstacles = extendObstaclesArray(state.AISData.ships)
    
    windDirection = state.windDirection
    runtime = 1

    # Run the planner multiple times and find the best one
    numRuns = 2
    solutions = []
    print("start: {} {}".format(start[0], start[1]))
    print("goal: {} {}".format(goal[0], goal[1]))
    print("dimensions: {} {} {} {}".format(dimensions[0], dimensions[1], dimensions[2], dimensions[3]))
    for o in obstacles:
        print("{}, {}, {}".format(o.x, o.y, o.radius))

    for i in range(numRuns):
        solutions.append(plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', windDirection, dimensions, start, goal, obstacles))

    solution = min(solutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())

    # Plot
    plot_path(solution.getSolutionPath().printAsMatrix(), dimensions, obstacles)

    return solution

def badPath(state, localPathSS, desiredHeading):
    # If sailing upwind or downwind, isBad
    if math.fabs(state.windDirection - desiredHeading) < math.radians(30) or math.fabs(state.windDirection - desiredHeading - math.radians(180)) < math.radians(30):
        rospy.loginfo_throttle(1, "Sailing upwind/downwind. Wind direction: {}. Desired Heading: {}".format(state.windDirection, desiredHeading))  # Prints every x seconds
        return True

    # Check if path will hit objects
    obstacles = [parse_obstacle("{},{},{}".format(ship.lat, ship.lon, ship.speed)) for ship in state.AISData.ships]
    if not hasNoCollisions(localPathSS, obstacles):
        rospy.loginfo("Going to hit obstacle.")
        return True
    else:
        rospy.loginfo("Not going to hit obstacle.")

    return False

def globalWaypointReached(position, globalWaypoint):
    radius = 2
    sailbot = (position.lat, position.lon)
    waypt = (globalWaypoint.lat, globalWaypoint.lon)
    return great_circle(sailbot, waypt) < radius

def localWaypointReached(position, localPath, localPathIndex):
    previousWaypoint = latlon(float(localPath[localPathIndex - 1].getY()), float(localPath[localPathIndex - 1].getX()))
    localWaypoint = latlon(float(localPath[localPathIndex].getY()), float(localPath[localPathIndex].getX()))
    isStartNorth = localWaypoint.lat < previousWaypoint.lat 
    isStartEast = localWaypoint.lon < previousWaypoint.lon
    tangentSlope = (localWaypoint.lat - previousWaypoint.lat) / (localWaypoint.lon - previousWaypoint.lon)
    normalSlope = -1/tangentSlope
    startX = previousWaypoint.lon - localWaypoint.lon
    startY = previousWaypoint.lat - localWaypoint.lat 
    boatX = position.lon - localWaypoint.lon 
    boatY = position.lat - localWaypoint.lat
    '''
    plt.xlim(-200, 200)
    plt.ylim(-200, 200)
    plt.plot([0], [0], marker = 'o', markersize=10, color="red")
    plt.plot([startX], [startY], marker="o", markersize=10, color="green")
    plt.plot([boatX], [boatY], marker = "o", markersize=10, color = "black")
    xvalues = [0, startX] 
    yvalues = [0, startY]
    plt.plot(xvalues, yvalues, "-g")
    x = np.linspace(-200, 200, 100)
    y = normalSlope * x
    plt.plot(x, y, '-r')
    y = tangentSlope * x
    plt.plot(x, y, '-b')
    plt.show()
    '''
    
    y = lambda x: normalSlope * x
    x = lambda y: y / float(normalSlope)
    
    if isStartNorth: 
        if boatY < y(boatX):
            return True
    elif boatY > y(boatX):
        return True

    if isStartEast: 
        if boatX < x(boatY):
            return True
    elif boatX > x(boatY):
        return True

    return False

def timeLimitExceeded(lastTimePathCreated):
    secondsLimit = 5
    return time.time() - lastTimePathCreated > secondsLimit

def getLocalWaypointLatLon(localPathSS, localPathIndex):
    # If local path is empty, return (0, 0)
    if len(localPathSS.getSolutionPath().getStates()) == 0:
        rospy.loginfo("Local path is empty.")
        rospy.loginfo("Setting localWaypoint to be (0, 0).")
        return latlon(0, 0)

    # If index out of range, return last waypoint in path
    if localPathIndex >= len(localPathSS.getSolutionPath().getStates()):
        rospy.loginfo("Local path index is out of range: index = {} len(localPath) = {}".format(localPathIndex, len(localPathSS.getSolutionPath().getStates())))
        rospy.loginfo("Setting localWaypoint to be the last element of the localPath")
        localWaypoint = localPathSS.getSolutionPath().getStates()[len(localPathSS.getSolutionPath().getStates())]

    # If index in range, return the correct waypoint
    else:
        localWaypoint = localPathSS.getSolutionPath().getStates()[localPathIndex]

    localWaypointLatLon = latlon()
    localWaypointLatLon.lat = localWaypoint.getY()
    localWaypointLatLon.lon = localWaypoint.getX()
    return localWaypointLatLon

# this will give initial bearing on a great-circle path
#if we keep local waypoints close enough to each other it approx the final bearing
def getDesiredHeading(position, localWaypoint):
    term1 = math.sin(localWaypoint.lon - position.lon) * math.cos(localWaypoint.lat)
    term2 = math.cos(position.lat) * math.sin(localWaypoint.lat) - math.sin(position.lat) * math.cos(localWaypoint.lat)*math.cos(localWaypoint.lon - position.lon)
    bearing = math.degrees(math.atan2(term1, term2))
    
    #this changes the bearing into a heading assuming 0 degrees pointing east for heading measurements
    heading = bearing - 90
    if (heading < 0):
        heading += 360
    return heading
    

def extendObstaclesArray(aisArray):
#assuming speed in km/h
    obstacles = []
    timeToLoc = 10  # change this value when deciding how much to extend obstacles
    radius = 0.2    # also change this to account for the width of the obstacle 
    toKMscale= 1.0 / 110   #approximate scaling factor of latlon to km
    spacing = 0.05 # can be changed to change width between obstacles

    for aisData in aisArray:
        if aisData.heading == 90 or aisData.heading == 270:
            endLat = aisData.lat + aisData.speed * toKMscale * timeToLoc
            yRange = np.arange(aisData.lat, endLat, spacing)
            for y in yRange:
                obstacles.append(Obstacle(aisData.lon, y, radius))
        else:
            
            isHeadingWest = aisData.heading < 270 and aisData.heading > 90
            slope = math.tan(math.radians(aisData.heading))

            if aisData.lon > 0:
                b = aisData.lat + slope * -math.fabs(aisData.lon)
            else:
                b = aisData.lat + slope * math.fabs(aisData.lon)
            xDistTravelled =  math.fabs(aisData.speed * toKMscale * timeToLoc * math.cos(math.radians(aisData.heading)))
            y = lambda x: slope * x + b 
            if isHeadingWest:
                endLon = aisData.lon - xDistTravelled
                xRange = np.arange(endLon, aisData.lon, spacing)
            else:
                endLon = aisData.lon + xDistTravelled
                xRange = np.arange(aisData.lon, endLon, spacing)
            for x in xRange:
                obstacles.append(Obstacle(x, y(x), radius))
    return obstacles            
