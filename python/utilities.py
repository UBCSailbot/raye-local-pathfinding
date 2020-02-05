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
from local_pathfinding.msg import latlon


def latlonToXY(latlon, relativeLatlon):
    x = distance((relativeLatlon.lat, relativeLatlon.lon), (relativeLatlon.lat, latlon.lon)).kilometers
    y = distance((relativeLatlon.lat, relativeLatlon.lon), (latlon.lat, relativeLatlon.lon)).kilometers
    if relativeLatlon.lon > latlon.lon:
        x = -x
    if relativeLatlon.lat > latlon.lat:
        y = -y
    return [x,y]

def XYToLatlon(xy, relativeLatlon):
    NORTH = 0
    EAST = 90
    SOUTH = 180
    WEST = 270
    x_distance = geopy.distance.distance(kilometers = xy[0])
    y_distance = geopy.distance.distance(kilometers = xy[1])
    destination = x_distance.destination(point=(relativeLatlon.lat, relativeLatlon.lon), bearing=EAST)
    destination = y_distance.destination(point=(destination.latitude, destination.longitude), bearing=NORTH)
    return latlon(destination.latitude, destination.longitude)

def createLocalPathSS(state):
    ou.setLogLevel(ou.LOG_WARN)

    # Get setup parameters from state
    # Convert all latlons to NE in km wrt boat position
    start = [0, 0]
    goal = latlonToXY(state.globalWaypoint, state.position)
    extra = 10   # Extra length to show more in the plot
    dimensions = [min(start[0], goal[0]) - extra, min(start[1], goal[1]) - extra, max(start[0], goal[0]) + extra, max(start[1], goal[1]) + extra]
    ships = [latlonToXY(latlon(ship.lat, ship.lon), state.position) for ship in state.AISData.ships]
    obstacles = [parse_obstacle("{},{},{}".format(ship[0], ship[1], 1)) for ship in ships]
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

    try:
        solution = min(solutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())
    except:
        print("Solution did not work")
        print("solutions[0] = {}".format(solutions[0]))
    solution.getSolutionPath().interpolate(10)

    # Plot in km units, with (0,0) being the start
    plot_path(solution.getSolutionPath().printAsMatrix(), dimensions, obstacles)

    # Plot in latlon units
    localPath = getLocalPath(solution, state.position)
    obstacles = [parse_obstacle("{},{},{}".format(ship.lon, ship.lat, 0.001)) for ship in state.AISData.ships]
    plot_path_2(localPath, obstacles)

    return solution, state.position

def getLocalPath(localPathSS, position):
    localPath = []
    for state in localPathSS.getSolutionPath().getStates():
        xy = (state.getX(), state.getY())
        localPath.append(XYToLatlon(xy, position))

    return localPath

def badPath(state, localPathSS, referenceLatlon, desiredHeading):
    # If sailing upwind or downwind, isBad
    if math.fabs(state.windDirection - desiredHeading) < math.radians(30) or math.fabs(state.windDirection - desiredHeading - math.radians(180)) < math.radians(30):
        rospy.loginfo_throttle(1, "Sailing upwind/downwind. Wind direction: {}. Desired Heading: {}".format(state.windDirection, desiredHeading))  # Prints every x seconds
        return True

    # Check if path will hit objects
    ships = [latlonToXY(latlon(ship.lat, ship.lon), referenceLatlon) for ship in state.AISData.ships]
    obstacles = [parse_obstacle("{},{},{}".format(ship[0], ship[1], 1)) for ship in ships]
    if not hasNoCollisions(localPathSS, obstacles):
        rospy.loginfo("Going to hit obstacle.")
        return True

    return False

def globalWaypointReached(position, globalWaypoint):
    radius = 20  # km
    sailbot = (position.lat, position.lon)
    waypt = (globalWaypoint.lat, globalWaypoint.lon)
    return distance(sailbot, waypt).kilometers < radius

def localWaypointReached(position, localPath, localPathIndex):
    previousWaypoint = latlon(float(localPath[localPathIndex - 1].lat), float(localPath[localPathIndex - 1].lon))
    localWaypoint = latlon(float(localPath[localPathIndex].lat), float(localPath[localPathIndex].lon))
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
    secondsLimit = 500
    return time.time() - lastTimePathCreated > secondsLimit

def getLocalWaypointLatLon(localPath, localPathIndex):
    # If local path is empty, return (0, 0)
    if len(localPath) == 0:
        rospy.loginfo("Local path is empty.")
        rospy.loginfo("Setting localWaypoint to be (0, 0).")
        return latlon(0, 0)

    # If index out of range, return last waypoint in path
    if localPathIndex >= len(localPath):
        rospy.loginfo("Local path index is out of range: index = {} len(localPath) = {}".format(localPathIndex, len(localPath)))
        rospy.loginfo("Setting localWaypoint to be the last element of the localPath")
        localPathIndex = len(localPath) - 1
        return localPath[localPathIndex]

    # If index in range, return the correct waypoint
    else:
        return localPath[localPathIndex]

# this will give initial bearing on a great-circle path
#if we keep local waypoints close enough to each other it approx the final bearing
def getDesiredHeading(position, localWaypoint):
    xy = latlonToXY(localWaypoint, position)
    return math.degrees(math.atan2(xy[1], xy[0]))

# Example code of how this class works.
if __name__ == '__main__':
    print("********************* Testing latlonToXY and XYToLatlon methods *********************")
    start = (49.263022, -123.023447)
    end = (47.7984, -125.3319)
    startLatlon = latlon(start[0], start[1])
    endLatlon = latlon(end[0], end[1])
    print("Start: {} End: {}".format(start, end))
    startEndDistance = distance((endLatlon.lat, endLatlon.lon), (startLatlon.lat, startLatlon.lon))
    print("Distance between start and end {}".format(startEndDistance))
    print("")
    xy = latlonToXY(endLatlon, startLatlon)
    print("XY between start and end is {}".format(xy))
    calculatedEndLatlon = XYToLatlon(xy, latlon(start[0], start[1]))
    print("End which is {} from {} is: \n({}, {}) Expected: ({}, {})".format(xy, start, calculatedEndLatlon.lat, calculatedEndLatlon.lon, endLatlon.lat, endLatlon.lon))
    endCalculatedEndDistance = distance((endLatlon.lat, endLatlon.lon), (calculatedEndLatlon.lat, calculatedEndLatlon.lon))
    print("Distance between end and calculatedEnd {}".format(endCalculatedEndDistance))
    print("***********************")
    print("Percent error: (distance(end, calculatedEnd) / distance(end, start)) = {}%".format(endCalculatedEndDistance/startEndDistance * 100))


