#!/usr/bin/env python
from ompl import util as ou
import rospy
import time
from plotting import plot_path
from updated_geometric_planner import plan, Obstacle, hasNoCollisions
from cli import parse_obstacle
import math 
from geopy.distance import great_circle
import geopy.distance
from local_pathfinding.msg import latlon


def latlonToXY(latlon, relativeLatlon):
    x = great_circle((relativeLatlon.lat, relativeLatlon.lon), (relativeLatlon.lat, latlon.lon)).kilometers
    y = great_circle((relativeLatlon.lat, relativeLatlon.lon), (latlon.lat, relativeLatlon.lon)).kilometers
    if relativeLatlon.lon > latlon.lon:
        x = -x
    if relativeLatlon.lat > latlon.lat:
        y = -y
    return [x,y]

def XYToLatlon(xy, relativeLatlon):
    x_distance = geopy.distance.VincentyDistance(kilometers = xy[0])
    y_distance = geopy.distance.VincentyDistance(kilometers = xy[1])
    destination = x_distance.destination(point=(relativeLatlon.lat, relativeLatlon.lon), bearing=0)
    destination = y_distance.destination(point=(destination.latitude, destination.longitude), bearing=0)
    return latlon(destination.latitude, destination.longitude)

def createLocalPathSS(state):
    ou.setLogLevel(ou.LOG_WARN)

    # Get setup parameters from state
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

    solution = min(solutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())
    solution.getSolutionPath().interpolate(10)

    # Plot
    plot_path(solution.getSolutionPath().printAsMatrix(), dimensions, obstacles)

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
    else:
        rospy.loginfo("Not going to hit obstacle.")

    return False

def globalWaypointReached(position, globalWaypoint):
    radius = 2
    sailbot = (position.lat, position.lon)
    waypt = (globalWaypoint.lat, globalWaypoint.lon)
    return great_circle(sailbot, waypt) < radius

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
    secondsLimit = 5
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
        return localPath[len(localPath) - 1]

    # If index in range, return the correct waypoint
    else:
        return localPath[localPathIndex]

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

# Example code of how this class works.
if __name__ == '__main__':
    print(latlonToXY(latlon(49.264766, -123.220042), latlon(49.263022, -123.023447)))
    d = geopy.distance.VincentyDistance(kilometers = 1)
    destination = d.destination(point=(49.264766, -123.220042), bearing=0)
    print("Start: {} Destination: {}".format((49.264766, -123.220042), (destination.latitude, destination.longitude)))
    print(great_circle(destination, (49.264766, -123.220042)))
