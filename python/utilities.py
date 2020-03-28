#!/usr/bin/env python
from ompl import util as ou
import rospy
import time
from plotting import plot_path, plot_path_2
from updated_geometric_planner import plan, Obstacle, hasObstacleOnPath
from planner_helpers import isUpwind, isDownwind
from cli import parse_obstacle
import math 
from geopy.distance import distance
import geopy.distance
from local_pathfinding.msg import latlon, AISShip, AISMsg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from Sailbot import BoatState

# Location constants
PORT_RENFREW_LATLON = latlon(48.5, -124.8)
MAUI_LATLON = latlon(20.0, -156.0)

# Constants
AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM = 3
GLOBAL_WAYPOINT_REACHED_RADIUS_KM = 5
PATH_UPDATE_TIME_LIMIT_SECONDS = 7200
MAX_ALLOWABLE_PATHFINDING_RUNTIME_SECONDS = 60

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

# Constants for modeling AIS boats
AIS_BOAT_RADIUS_KM = 0.2
AIS_BOAT_CIRCLE_SPACING_KM = AIS_BOAT_RADIUS_KM * 1.5  # Distance between circles that make up an AIS boat

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

def isValid(xy, obstacles):
    delta = 0.001
    for obstacle in obstacles:
        print("center point", (obstacle.x, obstacle.y))
        print("checking if pt is valid at:", xy)
        distance_center_to_boat = math.sqrt((xy[0] - obstacle.x) ** 2 + (xy[1] - obstacle.y) ** 2)
        angle_center_to_boat = math.degrees(math.atan2(xy[1] - obstacle.y, xy[0] - obstacle.x))
        angle_center_to_boat = (angle_center_to_boat + 360) % 360
        print("angle_center_to_boat", angle_center_to_boat)
        
        a = obstacle.width * 0.5
        b = obstacle.height * 0.5
        
        #t_param = math.atan2(a * (xy[1] - obstacle.y), b * (xy[0] - obstacle.x))
        #edge_pt = ellipseFormula(obstacle, t_param) 
        edge_pt = ellipseFormula(obstacle, math.radians(angle_center_to_boat))
        print(edge_pt)
        distance_to_edge = math.sqrt((edge_pt[0] - obstacle.x) ** 2 +  (edge_pt[1] - obstacle.y) ** 2)
        print("dist_to_edge_", distance_to_edge)
        print("distance_center_to_boat", distance_center_to_boat)
        #ax = plt.gca()
        #ax.add_patch(patches.Ellipse((obstacle.x, obstacle.y), obstacle.width, obstacle.height, obstacle.angle))
        #plt.plot([obstacle.x], [obstacle.y], marker='o', markersize=2, color="green")
        #plt.plot([xy[0]], [xy[1]], marker='o', markersize=2, color="red")
        #plt.plot([edge_pt[0]], [edge_pt[1]], marker='o', markersize=2, color="yellow")
        #plt.show()

        if distance_center_to_boat < distance_to_edge or math.fabs(distance_to_edge - distance_center_to_boat) <= delta: 
            return False
    return True

def ellipseFormula(obstacle, t):
    edge_pt = np.array([obstacle.x, obstacle.y]) + 0.5 * obstacle.width * math.cos(t) * np.array([math.cos(math.radians(obstacle.angle)), math.sin(math.radians(obstacle.angle))]) + 0.5 * obstacle.height * math.sin(t) * np.array([-math.sin(math.radians(obstacle.angle)), math.cos(math.radians(obstacle.angle))]) 
    return edge_pt

def ellipseFormula2(obstacle, angle):
    a = obstacle.width * 0.5
    b = obstacle.height * 0.5
    x = a * math.cos(angle) * math.cos(math.radians(obstacle.angle)) - b * math.sin(angle) * math.sin(math.radians(obstacle.angle))
    y = a * math.cos(angle) * math.sin(math.radians(obstacle.angle)) + b * math.sin(angle) * math.cos(math.radians(obstacle.angle))
    return [x, y]

def plotPathfindingProblem(globalWindDirectionDegrees, dimensions, start, goal, obstacles, headingDegrees, amountObstaclesShrinked):
    # Clear plot if already there
    plt.cla()

    # Create plot with start and goal
    markersize = min(dimensions[2]-dimensions[0], dimensions[3]-dimensions[1]) / 2
    plt.ion()
    axes = plt.gca()
    goalPlot, = axes.plot(goal[0], goal[1], marker='*', color='y', markersize=markersize)                          # Yellow star
    startPlot, = axes.plot(start[0], start[1], marker=(3,0,headingDegrees - 90), color='r', markersize=markersize) # Red triangle with correct heading. The (-90) is because the triangle default heading 0 points North, but this heading has 0 be East.

    # Setup plot xy limits and labels
    axes.set_xlim(dimensions[0], dimensions[2])
    axes.set_ylim(dimensions[1], dimensions[3])
    plt.grid(True)
    axes.set_xlabel('X distance to position (km)')
    axes.set_ylabel('Y distance to position (km)')
    axes.set_title('Setup of pathfinding problem (amountObstaclesShrinked = {})'.format(amountObstaclesShrinked))

    # Add boats and wind speed arrow
    for ship in obstacles:
        axes.add_patch(patches.Ellipse((ship.x, ship.y), ship.width, ship.height, ship.angle))

    arrowLength = min(dimensions[2]-dimensions[0], dimensions[3]-dimensions[1]) / 15
    arrowCenter = (dimensions[0] + 1.5*arrowLength, dimensions[3] - 1.5*arrowLength)
    arrowStart = (arrowCenter[0] - 0.5*arrowLength*math.cos(math.radians(globalWindDirectionDegrees)), arrowCenter[1] - 0.5*arrowLength*math.sin(math.radians(globalWindDirectionDegrees)))
    windDirection = patches.FancyArrow(arrowStart[0], arrowStart[1], arrowLength*math.cos(math.radians(globalWindDirectionDegrees)), arrowLength*math.sin(math.radians(globalWindDirectionDegrees)), width=arrowLength/4)
    axes.add_patch(windDirection)

    # Draw plot
    plt.draw()
    plt.pause(0.001)

def createLocalPathSS(state, runtimeSeconds=3, numRuns=3, plot=False):
    ou.setLogLevel(ou.LOG_WARN)

    # Get setup parameters from state for ompl plan()
    # Convert all latlons to NE in km wrt referenceLatlon
    referenceLatlon = state.position
    start = latlonToXY(state.position, referenceLatlon)
    goal = latlonToXY(state.globalWaypoint, referenceLatlon)
    extraKm = 10   # Extra length to allow wider solution space
    dimensions = [min(start[0], goal[0]) - extraKm, min(start[1], goal[1]) - extraKm, max(start[0], goal[0]) + extraKm, max(start[1], goal[1]) + extraKm]
    obstacles = extendObstaclesArray(state.AISData.ships, state.position, state.speedKmph, referenceLatlon)
    globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)

    # If start or goal is invalid, shrink objects and re-run
    # TODO: Figure out if there is a better method to handle this case
    amountShrinked = 1.0
    shrinkFactor = 2.0
    while not isValid(start, obstacles) or not isValid(goal, obstacles):
        rospy.logerr("start or goal state is not valid")
        rospy.logerr("Shrinking obstacles by a factor of {}".format(shrinkFactor))
        for obstacle in obstacles:
            obstacle.width /= shrinkFactor
            obstacle.height /= shrinkFactor
        amountShrinked *= shrinkFactor
    if amountShrinked > 1.0000001:
        rospy.logwarn("Obstacles have been shrinked by factor of {}".format(amountShrinked))

    # Run the planner multiple times and find the best one
    rospy.loginfo("Running createLocalPathSS. runtimeSeconds: {}. numRuns: {}. Total time: {} seconds".format(runtimeSeconds, numRuns, runtimeSeconds*numRuns))

    # Create non-blocking plot showing the setup of the pathfinding problem. Useful to understand if the pathfinding problem is invalid or impossible
    if plot:
        plotPathfindingProblem(globalWindDirectionDegrees, dimensions, start, goal, obstacles, state.headingDegrees, amountShrinked)

    def isValidSolution(solution, referenceLatlon, state):
        if not solution.haveExactSolutionPath():
            rospy.logwarn("Solution does not have exact solution path")
            return False
        # TODO: Check if this is needed or redundant. Sometimes it seemed like exact solution paths kept having obstacles on them, so it kept re-running, but need to do more testing
        if obstacleOnPath(state=state, nextLocalWaypointIndex=1, localPathSS=solution, referenceLatlon=referenceLatlon):
            rospy.logwarn("Solution has obstacle on path")
            return False
        return True

    solutions = []
    for i in range(numRuns):
        # TODO: Incorporate globalWindSpeed into pathfinding?
        rospy.loginfo("Starting path-planning run number: {}".format(i))
        solution = plan(runtimeSeconds, "RRTStar", 'WeightedLengthAndClearanceCombo', globalWindDirectionDegrees, dimensions, start, goal, obstacles)
        if isValidSolution(solution, referenceLatlon, state):
            solutions.append(solution)

    # If no solutions found, re-run with larger runtime
    # TODO: Figure out if there is a better method to handle this case
    increaseRuntimeFactor = 2.0
    while len(solutions) == 0:
        rospy.logerr("No valid solutions found in {} seconds runtime".format(runtimeSeconds))
        runtimeSeconds *= increaseRuntimeFactor
        rospy.logerr("Attempting to rerun with longer runtime: {} seconds".format(runtimeSeconds))

        # TODO: Incorporate globalWindSpeed into pathfinding?
        solution = plan(runtimeSeconds, "RRTStar", 'WeightedLengthAndClearanceCombo', globalWindDirectionDegrees, dimensions, start, goal, obstacles)

        if isValidSolution(solution, referenceLatlon, state):
            solutions.append(solution)

        # If valid solution can't be found for large runtime, then just use the invalid solution
        elif runtimeSeconds * increaseRuntimeFactor >= MAX_ALLOWABLE_PATHFINDING_RUNTIME_SECONDS:
            rospy.logerr("No valid solution can be found. Using invalid solution.")
            solutions.append(solution)

    solution = min(solutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())

    # Set the average distance between waypoints
    localPathLengthKm = solution.getSolutionPath().length()
    numberOfLocalWaypoints = int(localPathLengthKm / AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM)
    solution.getSolutionPath().interpolate(numberOfLocalWaypoints)

    # Close any plots
    plt.close()
    return solution, referenceLatlon

def getLocalPath(localPathSS, referenceLatlon):
    # Convert localPathSS solution path (in km WRT reference) into list of latlons
    localPath = []
    for state in localPathSS.getSolutionPath().getStates():
        xy = (state.getX(), state.getY())
        localPath.append(XYToLatlon(xy, referenceLatlon))
    return localPath

def sailingUpwindOrDownwind(state, desiredHeadingDegrees):
    globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)

    if isDownwind(math.radians(globalWindDirectionDegrees), math.radians(desiredHeadingDegrees)):
        rospy.logwarn("Sailing downwind. Global Wind direction: {}. Desired Heading: {}".format(globalWindDirectionDegrees, desiredHeadingDegrees))
        return True

    elif isUpwind(math.radians(globalWindDirectionDegrees), math.radians(desiredHeadingDegrees)):
        rospy.logwarn("Sailing upwind. Global Wind direction: {}. Desired Heading: {}".format(globalWindDirectionDegrees, desiredHeadingDegrees))
        return True

    return False

def obstacleOnPath(state, nextLocalWaypointIndex, localPathSS, referenceLatlon):
    # Check if path will hit objects
    positionXY = latlonToXY(state.position, referenceLatlon)
    obstacles = extendObstaclesArray(state.AISData.ships, state.position, state.speedKmph, referenceLatlon)
    if hasObstacleOnPath(positionXY, nextLocalWaypointIndex, localPathSS, obstacles):
        rospy.logwarn("Obstacle on path!")
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

def timeLimitExceeded(lastTimePathCreated, speedup):
    # Shorter time limit when there is speedup
    pathUpdateTimeLimitSecondsSpeedup = PATH_UPDATE_TIME_LIMIT_SECONDS / speedup
    return time.time() - lastTimePathCreated > pathUpdateTimeLimitSecondsSpeedup

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

def extendObstaclesArray(aisArray, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
    obstacles = []
    for aisData in aisArray:
        aisX, aisY = latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)

        # Calculate length to extend boat
        MAX_TIME_TO_LOC_HOURS = 10  # Do not extend objects more than 10 hours distance
        distanceToBoatKm = distance((aisData.lat, aisData.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
        if sailbotSpeedKmph == 0 or distanceToBoatKm / sailbotSpeedKmph > MAX_TIME_TO_LOC_HOURS:
            timeToLocHours = MAX_TIME_TO_LOC_HOURS
        else:
            timeToLocHours = distanceToBoatKm / sailbotSpeedKmph
        extendBoatLengthKm = aisData.speedKmph * timeToLocHours

        if extendBoatLengthKm == 0:
            obstacles.append(Obstacle(aisX, aisY, AIS_BOAT_RADIUS_KM, AIS_BOAT_RADIUS_KM, 0))

        width = extendBoatLengthKm
        height = AIS_BOAT_RADIUS_KM
        angle = aisData.headingDegrees
        xy = [aisX + extendBoatLengthKm * math.cos(math.radians(angle)) * 0.5, aisY + extendBoatLengthKm * math.sin(math.radians(angle)) * 0.5]
        obstacles.append(Obstacle(xy[0], xy[1], width, height, angle))
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

