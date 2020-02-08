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
from local_pathfinding.msg import latlon, AIS_ship


def latlonToXY(latlon, referenceLatlon):
    x = distance((referenceLatlon.lat, referenceLatlon.lon), (referenceLatlon.lat, latlon.lon)).kilometers
    y = distance((referenceLatlon.lat, referenceLatlon.lon), (latlon.lat, referenceLatlon.lon)).kilometers
    if referenceLatlon.lon > latlon.lon:
        x = -x
    if referenceLatlon.lat > latlon.lat:
        y = -y
    return [x,y]

def XYToLatlon(xy, referenceLatlon):
    NORTH = 0
    EAST = 90
    SOUTH = 180
    WEST = 270
    x_distance = geopy.distance.distance(kilometers = xy[0])
    y_distance = geopy.distance.distance(kilometers = xy[1])
    destination = x_distance.destination(point=(referenceLatlon.lat, referenceLatlon.lon), bearing=EAST)
    destination = y_distance.destination(point=(destination.latitude, destination.longitude), bearing=NORTH)
    return latlon(destination.latitude, destination.longitude)

def createLocalPathSS(state):
    ou.setLogLevel(ou.LOG_WARN)

    # Get setup parameters from state for ompl plan()
    # Convert all latlons to NE in km wrt boat position
    start = [0, 0]
    goal = latlonToXY(state.globalWaypoint, state.position)
    extra = 10   # Extra length to show more in the plot
    dimensions = [min(start[0], goal[0]) - extra, min(start[1], goal[1]) - extra, max(start[0], goal[0]) + extra, max(start[1], goal[1]) + extra]
    ships = [latlonToXY(latlon(ship.lat, ship.lon), state.position) for ship in state.AISData.ships]
    obstacles = [parse_obstacle("{},{},{}".format(ship[0], ship[1], 1)) for ship in ships]
    windDirection = state.windDirection
    runtime = 5

    # Run the planner multiple times and find the best one
    numRuns = 1
    rospy.loginfo("Running createLocalPathSS. runTime: {}. numRuns: {}. Total time: {} seconds".format(runtime, numRuns, runtime*numRuns))
    solutions = []
    for i in range(numRuns):
        rospy.loginfo("Starting run {}".format(i))
        solutions.append(plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', windDirection, dimensions, start, goal, obstacles))

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

    # Plot in km units, with (0,0) being the start
    # plot_path(solution.getSolutionPath().printAsMatrix(), dimensions, obstacles)

    # Plot in latlon units
    # localPath = getLocalPath(solution, state.position)
    # obstacles = [parse_obstacle("{},{},{}".format(ship.lon, ship.lat, 0.001)) for ship in state.AISData.ships]
    # plot_path_2(localPath, obstacles)

    return solution, state.position

def getLocalPath(localPathSS, referenceLatlon):
    # Convert localPathSS solution path (in km WRT reference) into list of latlons
    localPath = []
    for state in localPathSS.getSolutionPath().getStates():
        xy = (state.getX(), state.getY())
        localPath.append(XYToLatlon(xy, referenceLatlon))
    return localPath

def badPath(state, localPathSS, referenceLatlon, desiredHeading):
    # If sailing upwind or downwind, isBad
    if math.fabs(state.windDirection - desiredHeading) < math.radians(30) or math.fabs(state.windDirection - desiredHeading - math.radians(180)) < math.radians(30):
        rospy.logwarn("Sailing upwind/downwind. Wind direction: {}. Desired Heading: {}".format(state.windDirection, desiredHeading))
        return True

    # Check if path will hit objects
    ships = [latlonToXY(latlon(ship.lat, ship.lon), referenceLatlon) for ship in state.AISData.ships]
    obstacles = [parse_obstacle("{},{},{}".format(ship[0], ship[1], 1)) for ship in ships]
    if not hasNoCollisions(localPathSS, obstacles):
        rospy.logwarn("Going to hit obstacle.")
        return True

    return False

def globalWaypointReached(position, globalWaypoint):
    radius = 200  # km
    sailbot = (position.lat, position.lon)
    waypt = (globalWaypoint.lat, globalWaypoint.lon)
    dist = distance(sailbot, waypt).kilometers
    print("Distance to globalWaypoint is {}".format(dist))
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

# this will give initial bearing on a great-circle path
#if we keep local waypoints close enough to each other it approx the final bearing
def getDesiredHeading(position, localWaypoint):
    xy = latlonToXY(localWaypoint, position)
    return math.degrees(math.atan2(xy[1], xy[0]))

def extendObstacles(aisData, timeToLoc):
#modify to take in array of ships and execute below for all ships in array
    obstacles = []
    isHeadingWest = aisData.heading < 270 and aisData.heading > 90
    radius = 0.2
    spacing = 0.1
    slope = math.tan(math.radians(aisData.heading))
    print slope
    if aisData.lon > 0:
        b = aisData.lat + slope * -math.fabs(aisData.lon) #check math
    else:
        b = aisData.lat + slope * math.fabs(aisData.lon)
    print b
    xDistTravelled =  math.fabs(aisData.speed * timeToLoc * math.cos(math.radians(aisData.heading)))
    y = lambda x: slope * x + b 
    if isHeadingWest:
        endLon = aisData.lon - xDistTravelled
        xRange = np.arange(endLon, aisData.lon, spacing)
        start = len(xRange) - 1
    else:
        endLon = aisData.lon + xDistTravelled
        xRange = np.arange(aisData.lon, endLon, spacing)
        start = 0
    for x in xRange:
        obstacles.append(Obstacle(x, y(x), radius))
    obstacles = [parse_obstacle("{},{},{}".format(obstacle.x, obstacle.y, obstacle.radius)) for obstacle in obstacles]
    x = np.linspace(aisData.lon, endLon, 100)
    y = y(x) 
    plt.plot(x, y, '-r')
    ax = plt.gca()
    for obstacle in obstacles:
        ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))
    ax.add_patch(plt.Circle((obstacles[start].x, obstacles[start].y), radius=obstacles[start].radius)).set_color('green')
    plt.show()
    return obstacles

# Example code of how to use some of these methods.
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


