#!/usr/bin/env python
from ompl import util as ou
from ompl import geometric as og
import rospy
import time
import sys
from plotting import plot_path, plot_path_2
from updated_geometric_planner import plan, indexOfObstacleOnPath
from planner_helpers import isUpwind, isDownwind
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
GLOBAL_WAYPOINT_REACHED_RADIUS_KM = 10
PATH_UPDATE_TIME_LIMIT_SECONDS = 7200

# Pathfinding constants
MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS = 120.0
INCREASE_RUNTIME_FACTOR = 1.5

# Scale NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES and NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND to change based on waypoint distance
LOOK_AHEAD_FOR_OBSTACLES_KM = 20
NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES = int(math.ceil(LOOK_AHEAD_FOR_OBSTACLES_KM / AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM))
LOOK_AHEAD_FOR_UPWIND_DOWNWIND_KM = 10
NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND = int(math.ceil(LOOK_AHEAD_FOR_UPWIND_DOWNWIND_KM / AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM))

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

# Upwind downwind detection
UPWIND_DOWNWIND_COUNTER_LIMIT = 3

# Constants for pathfinding updates
COST_THRESHOLD = 12000
MAX_ALLOWABLE_DISTANCE_FINAL_WAYPOINT_TO_GOAL_KM = 5

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

def plotPathfindingProblem(globalWindDirectionDegrees, dimensions, start, goal, obstacles, headingDegrees, amountObstaclesShrinked):
    # Clear plot if already there
    plt.cla()

    # Create plot with start and goal
    x_min, y_min, x_max, y_max = dimensions
    markersize = min(x_max - x_min, y_max - y_min) / 2
    plt.ion()
    axes = plt.gca()
    goalPlot, = axes.plot(goal[0], goal[1], marker='*', color='y', markersize=markersize)                          # Yellow star
    startPlot, = axes.plot(start[0], start[1], marker=(3,0,headingDegrees - 90), color='r', markersize=markersize) # Red triangle with correct heading. The (-90) is because the triangle default heading 0 points North, but this heading has 0 be East.

    # Setup plot xy limits and labels
    axes.set_xlim(x_min, x_max)
    axes.set_ylim(y_min, y_max)
    axes.set_aspect('equal')
    plt.grid(True)
    axes.set_xlabel('X distance to position (km)')
    axes.set_ylabel('Y distance to position (km)')
    axes.set_title('Setup of pathfinding problem (amountObstaclesShrinked = {})'.format(amountObstaclesShrinked))

    # Add boats and wind speed arrow
    for obstacle in obstacles:
        obstacle.addPatch(axes)

    arrowLength = min(dimensions[2]-dimensions[0], dimensions[3]-dimensions[1]) / 15
    arrowCenter = (dimensions[0] + 1.5*arrowLength, dimensions[3] - 1.5*arrowLength)
    arrowStart = (arrowCenter[0] - 0.5*arrowLength*math.cos(math.radians(globalWindDirectionDegrees)), arrowCenter[1] - 0.5*arrowLength*math.sin(math.radians(globalWindDirectionDegrees)))
    windDirection = patches.FancyArrow(arrowStart[0], arrowStart[1], arrowLength*math.cos(math.radians(globalWindDirectionDegrees)), arrowLength*math.sin(math.radians(globalWindDirectionDegrees)), width=arrowLength/4)
    axes.add_patch(windDirection)

    # Draw plot
    plt.draw()
    plt.pause(0.001)

def createLocalPathSS(state, runtimeSeconds=2, numRuns=4, plot=False):
    def getXYLimits(start, goal, extraLengthFraction=0.6):
        # Calculate extra length to allow wider solution space
        width = math.fabs(goal[0] - start[0])
        height = math.fabs(goal[1] - start[1])
        extraKm = extraLengthFraction * max(width, height)

        xMin = min(start[0], goal[0]) - extraKm
        yMin = min(start[1], goal[1]) - extraKm
        xMax = max(start[0], goal[0]) + extraKm
        yMax = max(start[1], goal[1]) + extraKm
        return [xMin, yMin, xMax, yMax]

    ou.setLogLevel(ou.LOG_WARN)

    # Get setup parameters from state for ompl plan()
    # Convert all latlons to NE in km wrt referenceLatlon
    referenceLatlon = state.globalWaypoint
    start = latlonToXY(state.position, referenceLatlon)
    goal = latlonToXY(state.globalWaypoint, referenceLatlon)
    dimensions = getXYLimits(start, goal)
    obstacles = getObstacles(state.AISData.ships, state.position, state.speedKmph, referenceLatlon)
    globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)

    # If start or goal is invalid, shrink objects and re-run
    # TODO: Figure out if there is a better method to handle this case
    amountShrinked = 1.0
    shrinkFactor = 2.0
    while not isValid(start, obstacles) or not isValid(goal, obstacles):
        rospy.logerr("start or goal state is not valid")
        rospy.logerr("Shrinking obstacles by a factor of {}".format(shrinkFactor))
        for obstacle in obstacles:
            obstacle.shrink(shrinkFactor)
        amountShrinked *= shrinkFactor
    if amountShrinked > 1.0000001:
        rospy.logerr("Obstacles have been shrinked by factor of {}".format(amountShrinked))

    # Run the planner multiple times and find the best one
    rospy.loginfo("Running createLocalPathSS. runtimeSeconds: {}. numRuns: {}. Total time: {} seconds".format(runtimeSeconds, numRuns, runtimeSeconds*numRuns))

    # Create non-blocking plot showing the setup of the pathfinding problem. Useful to understand if the pathfinding problem is invalid or impossible
    if plot:
        plotPathfindingProblem(globalWindDirectionDegrees, dimensions, start, goal, obstacles, state.headingDegrees, amountShrinked)

    def isValidSolution(solution, referenceLatlon, state):
        if not solution.haveExactSolutionPath():
            return False
        # TODO: Investigate if we should put a max cost threshold that makes convoluted paths unacceptable
        return True

    solutions = []
    invalidSolutions = []
    for i in range(numRuns):
        # TODO: Incorporate globalWindSpeed into pathfinding?
        rospy.loginfo("Starting path-planning run number: {}".format(i))
        solution = plan(runtimeSeconds, "RRTStar", 'WeightedLengthAndClearanceCombo', globalWindDirectionDegrees, dimensions, start, goal, obstacles)
        if isValidSolution(solution, referenceLatlon, state):
            solutions.append(solution)
        else:
            invalidSolutions.append(solution)

    # If no solutions found, re-run with larger runtime
    # TODO: Figure out if there is a better method to handle this case
    totalRuntimeSeconds = numRuns * runtimeSeconds
    while len(solutions) == 0:
        rospy.logwarn("No valid solutions found in {} seconds runtime".format(runtimeSeconds))
        runtimeSeconds *= INCREASE_RUNTIME_FACTOR
        totalRuntimeSeconds += runtimeSeconds

        # If valid solution can't be found for large runtime, then stop searching
        if totalRuntimeSeconds >= MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS:
            rospy.logwarn("No valid solution can be found in under {} seconds. Using invalid solution.".format(MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS))
            break

        rospy.logwarn("Attempting to rerun with longer runtime: {} seconds".format(runtimeSeconds))
        # TODO: Incorporate globalWindSpeed into pathfinding?
        solution = plan(runtimeSeconds, "RRTStar", 'WeightedLengthAndClearanceCombo', globalWindDirectionDegrees, dimensions, start, goal, obstacles)

        if isValidSolution(solution, referenceLatlon, state):
            solutions.append(solution)
        else:
            invalidSolutions.append(solution)

    # If no valid solutions found, use the best invalid one. Do not perform any path simplifying on invalid paths.
    if len(solutions) == 0:
        bestSolution = min(invalidSolutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())
        bestSolutionPath = solution.getSolutionPath()
        minCost = bestSolutionPath.cost(bestSolution.getOptimizationObjective()).value()
    else:

        # Find path with minimum cost. Can be either simplified or unsimplified path.
        # Need to recheck that simplified paths are valid before using
        minCost = sys.maxsize
        bestSolution = None
        bestSolutionPath = None
        for solution in solutions:
            # Check unsimplified path
            unsimplifiedPath = og.PathGeometric(solution.getSolutionPath())
            unsimplifiedCost = unsimplifiedPath.cost(solution.getOptimizationObjective()).value()
            if unsimplifiedCost < minCost:
                bestSolution = solution
                bestSolutionPath = unsimplifiedPath
                minCost = unsimplifiedCost

            # Check simplified path
            solution.simplifySolution()
            simplifiedPath = solution.getSolutionPath()
            simplifiedCost = simplifiedPath.cost(solution.getOptimizationObjective()).value()
            if simplifiedCost < minCost:
                # Double check that simplified path is valid
                nextLocalWaypointIndex = 1  # Start at first possible waypoint (0 is boat position)
                hasObstacleOnPath = obstacleOnPath(state, nextLocalWaypointIndex, solution, simplifiedPath, referenceLatlon)
                hasUpwindOrDownwindOnPath = upwindOrDownwindOnPath(state, nextLocalWaypointIndex, simplifiedPath, referenceLatlon)
                isStillValid = not hasObstacleOnPath and not hasUpwindOrDownwindOnPath
                if isStillValid:
                    bestSolution = solution
                    bestSolutionPath = simplifiedPath
                    minCost = simplifiedCost

    rospy.loginfo("Found solution with cost: {}".format(minCost))

    # Set the average distance between waypoints
    localPathLengthKm = bestSolutionPath.length()
    numberOfLocalWaypoints = int(localPathLengthKm / AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM)
    bestSolutionPath.interpolate(numberOfLocalWaypoints)

    # Close any plots
    plt.close()
    return bestSolution, bestSolutionPath, referenceLatlon

def getLocalPathLatlons(solutionPathObject, referenceLatlon):
    # Convert solution path (in km WRT reference) into list of latlons
    localPath = []
    for state in solutionPathObject.getStates():
        xy = (state.getX(), state.getY())
        localPath.append(XYToLatlon(xy, referenceLatlon))
    return localPath

def upwindOrDownwindOnPath(state, nextLocalWaypointIndex, solutionPathObject, referenceLatlon, numLookAheadWaypoints=None, showWarnings=False):
    # Default behavior when numLookAheadWaypoints is not given
    if numLookAheadWaypoints is None:
        numLookAheadWaypoints = len(solutionPathObject.getStates()) - nextLocalWaypointIndex
    # Handle bad input
    if nextLocalWaypointIndex + numLookAheadWaypoints > len(solutionPathObject.getStates()):
        numLookAheadWaypoints = len(solutionPathObject.getStates()) - nextLocalWaypointIndex

    # Calculate global wind from measured wind and boat state
    globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)

    # Get relevant waypoints (boat position first, then the next numLookAheadWaypoints startin from nextLocalWaypoint onwards)
    relevantWaypoints = []
    relevantWaypoints.append(latlonToXY(state.position, referenceLatlon))
    for waypointIndex in range(nextLocalWaypointIndex, nextLocalWaypointIndex + numLookAheadWaypoints):
        waypoint = solutionPathObject.getState(waypointIndex)
        relevantWaypoints.append([waypoint.getX(), waypoint.getY()])

    # Check relevantWaypoints for upwind or downwind sailing
    upwindOrDownwind = False
    for waypointIndex in range(1, len(relevantWaypoints)):
        # Calculate required heading between waypoints
        waypoint = relevantWaypoints[waypointIndex]
        prevWaypoint = relevantWaypoints[waypointIndex - 1]
        requiredHeadingDegrees = math.degrees(math.atan2(waypoint[1] - prevWaypoint[1], waypoint[0] - prevWaypoint[0]))

        if isDownwind(math.radians(globalWindDirectionDegrees), math.radians(requiredHeadingDegrees)):
            if showWarnings:
                rospy.logwarn("Downwind sailing on path. globalWindDirectionDegrees: {}. requiredHeadingDegrees: {}. waypointIndex: {}".format(globalWindDirectionDegrees, requiredHeadingDegrees, waypointIndex))
            upwindOrDownwind = True
            break

        elif isUpwind(math.radians(globalWindDirectionDegrees), math.radians(requiredHeadingDegrees)):
            if showWarnings:
                rospy.logwarn("Upwind sailing on path. globalWindDirectionDegrees: {}. requiredHeadingDegrees: {}. waypointIndex: {}".format(globalWindDirectionDegrees, requiredHeadingDegrees, waypointIndex))
            upwindOrDownwind = True
            break

    # Set counter to 0 on first use
    try:
        firstTime = upwindOrDownwindOnPath.counter is None
    except AttributeError:
        rospy.loginfo("Handling first time case in upwindOrDownwindOnPath()")
        upwindOrDownwindOnPath.counter = 0

    # Increment or reset counter
    if upwindOrDownwind:
        upwindOrDownwindOnPath.counter += 1
    else:
        upwindOrDownwindOnPath.counter = 0

    # Return true only if upwindOrDownwind for count times
    if upwindOrDownwindOnPath.counter >= UPWIND_DOWNWIND_COUNTER_LIMIT:
        upwindOrDownwindOnPath.counter = 0
        return True
    else:
        return False

def obstacleOnPath(state, nextLocalWaypointIndex, localPathSS, solutionPathObject, referenceLatlon, numLookAheadWaypoints=None, showWarnings=False):
    # Check if path will hit objects
    positionXY = latlonToXY(state.position, referenceLatlon)
    obstacles = getObstacles(state.AISData.ships, state.position, state.speedKmph, referenceLatlon)

    # Ensure nextLocalWaypointIndex + numLookAheadWaypoints is in bounds
    '''
    Let path = [(0,0), (1,1), (2,2), (3,3), (4,4)], len(path) = 5

    If: nextLocalWaypointIndex = 2, numLookAheadWaypoints = 2
    Then: care about waypoints (2,2), (3,3)
          len(path) >= nextLocalWaypointIndex + numLookAheadWaypoints, so valid

    If: nextLocalWaypointIndex = 2, numLookAheadWaypoints = 6
    Then: care about waypoints (2,2), (3,3), (4,4)
          len(path) < nextLocalWaypointIndex + numLookAheadWaypoints, so invalid
          update numLookAheadWaypoints to len(path) - nextLocalWaypointIndex = 3
    '''
    if numLookAheadWaypoints is None:
        numLookAheadWaypoints = len(solutionPathObject.getStates()) - nextLocalWaypointIndex
    if nextLocalWaypointIndex + numLookAheadWaypoints > len(solutionPathObject.getStates()):
        numLookAheadWaypoints = len(solutionPathObject.getStates()) - nextLocalWaypointIndex

    waypointIndexWithObstacle = indexOfObstacleOnPath(positionXY, nextLocalWaypointIndex, numLookAheadWaypoints, localPathSS, solutionPathObject, obstacles)
    if waypointIndexWithObstacle != -1:
        if showWarnings:
            rospy.logwarn("Obstacle on path. waypointIndexWithObstacle: {}".format(waypointIndexWithObstacle))
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

def getPathCostBreakdownString(optimizationObjective, solutionPathObject):
    # Assumes balanced optimization objective
    strings = []
    for i in range(optimizationObjective.getObjectiveCount()):
        objective = optimizationObjective.getObjective(i)
        weight = optimizationObjective.getObjectiveWeight(i)
        cost = solutionPathObject.cost(objective).value()
        strings.append("{}: Cost = {}. Weight = {}. Weighted Cost = {} |||| ".format(type(objective).__name__, cost, weight, cost * weight))
    strings.append("--------------------------------------------------- ")
    strings.append("{}: Total Cost = {}".format(type(optimizationObjective).__name__, solutionPathObject.cost(optimizationObjective).value()))

    output = ''.join(strings)
    return output

def isValid(xy, obstacles):
    for obstacle in obstacles:
        if not obstacle.isValid(xy):
            return False
    return True

class ObstacleInterface:
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        self.aisData = aisData
        self.sailbotPosition = sailbotPosition
        self.speedKmph = speedKmph
        self.referenceLatlon = referenceLatlon
        pass

    def __str__(self):
        pass

    def _extendObstacle(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        """ Extends obstacle based on speed and heading """
        pass

    def addPatch(self, axes):
        """ Return patch from matplotlib.patches """
        pass

    def isValid(self, xy):
        """ Checks validity of xy"""
        pass

    def clearance(self, posX, posY):
        """ Return distance from obstacle to (posX, posY)"""
        pass

    def shrink(self, shrinkFactor):
        """ Shrinks the obstacle by the shrink factor"""
        pass


class Ellipse(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
        self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)

    def __str__(self):
        return str((self.x, self.y, self.height, self.width, self.angle))

    def _extendObstacle(self, aisData, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
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
            width = AIS_BOAT_RADIUS_KM
        else:
            width = extendBoatLengthKm
        height = AIS_BOAT_RADIUS_KM
        angle = aisData.headingDegrees
        xy = [aisX + extendBoatLengthKm * math.cos(math.radians(angle)) * 0.5, aisY + extendBoatLengthKm * math.sin(math.radians(angle)) * 0.5]
        self.x, self.y = xy[0], xy[1]
        self.width, self.height = width, height
        self.angle = angle
    
    def isValid(self, xy):
        delta = 0.001
        x = xy[0] - self.x
        y = xy[1] - self.y
        x_ = math.cos(math.radians(self.angle)) * x + math.sin(math.radians(self.angle)) * y 
        y_ = -math.sin(math.radians(self.angle)) * x + math.cos(math.radians(self.angle)) * y 
        distance_center_to_boat = math.sqrt(x_ ** 2 + y_ ** 2)
        angle_center_to_boat = math.degrees(math.atan2(y_, x_))
        angle_center_to_boat = (angle_center_to_boat + 360) % 360 
    
        a = self.width * 0.5 
        b = self.height * 0.5 
    
        t_param = math.atan2(a * y_, b * x_) 
        edge_pt = self._ellipseFormula(t_param) 
        distance_to_edge = math.sqrt((edge_pt[0] - self.x) ** 2 +  (edge_pt[1] - self.y) ** 2)

        if distance_center_to_boat < distance_to_edge or math.fabs(distance_to_edge - distance_center_to_boat) <= delta: 
            return False
        return True
        
    def _ellipseFormula(self, t): 
        init_pt = np.array([self.x, self.y])
        a = 0.5 * self.width
        b = 0.5 * self.height
        rotation_col1 = np.array([math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle))]) 
        rotation_col2 = np.array([-math.sin(math.radians(self.angle)), math.cos(math.radians(self.angle))]) 
        edge_pt = init_pt + a * math.cos(t) * rotation_col1 + b * math.sin(t) * rotation_col2
        return edge_pt

    def addPatch(self, axes):
        axes.add_patch(patches.Ellipse((self.x, self.y), self.width, self.height, self.angle))

    def shrink(self, shrinkFactor):
        self.width /= shrinkFactor 
        self.height /= shrinkFactor 

    def clearance(self, posX, posY):
        return 1 

class Wedge(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
            ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
            self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)

    def __str__(self):
            return str((self.x, self.y, self.radius, self.theta1, self.theta2))
    
    def _extendObstacle(self, aisData, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        EXPAND_ANGLE = 0.5 
        MAX_TIME_TO_LOC_HOURS = 10  # Do not extend objects more than 10 hours distance

        aisX, aisY = latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)

        theta1 = aisData.headingDegrees - EXPAND_ANGLE
        theta2 = aisData.headingDegrees + EXPAND_ANGLE
        if theta1 < 0:
            theta1 +=360
        if theta2 > 360:
            theta2 -= 360

        distanceToBoatKm = distance((aisData.lat, aisData.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
        if sailbotSpeedKmph == 0 or distanceToBoatKm / sailbotSpeedKmph > MAX_TIME_TO_LOC_HOURS:
            timeToLocHours = MAX_TIME_TO_LOC_HOURS
        else:
            timeToLocHours = distanceToBoatKm / sailbotSpeedKmph

        radius = aisData.speedKmph * timeToLocHours
        self.x, self.y = aisX, aisY
        self.radius = radius
        self.theta1, self.theta2 = theta1, theta2

    def addPatch(self, axes):
        axes.add_patch(patches.Wedge((self.x, self.y), self.radius, self.theta1, self.theta2))

    def isValid(self, xy):
        INVALID_RADIUS_AROUND_START = 1
        angle = math.degrees(math.atan2(xy[1] - self.y, xy[0] - self.x))
        if angle < 0:
            angle += 360
        distance = math.sqrt((xy[1] - self.y) **2 + (xy[0] - self.x) ** 2)
        if distance < INVALID_RADIUS_AROUND_START:
            return False 
        if (angle > self.theta1 and angle < self.theta2 and distance < self.radius):
            return False
        return True

    def clearance(self, posX, posY):
        return 1

    def shrink(self, shrinkFactor):
        self.radius /= shrinkFactor 

class Circles(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
            ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
            self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)

    def isValid(self, xy):
        x, y = xy
        for obstacle in self.obstacles:
            if not obstacle.isValid(xy):
                return False
        return True

    def _extendObstacle(self, aisData, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        self.obstacles = []
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
            self.obstacles.append(Circle(aisX, aisY, AIS_BOAT_RADIUS_KM))


        if aisData.headingDegrees == 90 or aisData.headingDegrees == 270:
            if aisData.headingDegrees == 90:
                endY = aisY + extendBoatLengthKm
                yRange = np.arange(aisY, endY, AIS_BOAT_CIRCLE_SPACING_KM)
            if aisData.headingDegrees == 270:
                endY = aisY - extendBoatLengthKm
                yRange = np.arange(endY, aisY, AIS_BOAT_CIRCLE_SPACING_KM)
            for y in yRange:
                # Multiplier to increase size of circles showing where the boat will be in the future in range [1, 2]
                multiplier = 1 + abs(float(y - aisY) / (endY - aisY))
                self.obstacles.append(Circle(aisX, y, AIS_BOAT_RADIUS_KM * multiplier))
        else:
            isHeadingWest = aisData.headingDegrees < 270 and aisData.headingDegrees > 90
            slope = math.tan(math.radians(aisData.headingDegrees))
            dx = AIS_BOAT_CIRCLE_SPACING_KM / math.sqrt(1 + slope**2)

            if aisX > 0:
                b = aisY + slope * -math.fabs(aisX)
            else:
                b = aisY + slope * math.fabs(aisX)
            xDistTravelled =  math.fabs(extendBoatLengthKm * math.cos(math.radians(aisData.headingDegrees)))
            y = lambda x: slope * x + b 
            if isHeadingWest:
                endX = aisX - xDistTravelled
                xRange = np.arange(endX, aisX, dx)
            else:
                endX = aisX + xDistTravelled
                xRange = np.arange(aisX, endX, dx)
            for x in xRange:
                # Multiplier to increase size of circles showing where the boat will be in the future in range [1, 2]
                multiplier = 1 + abs(float(x - aisX) / (endX - aisX))
                self.obstacles.append(Circle(x, y(x), AIS_BOAT_RADIUS_KM * multiplier))

    def shrink(self, shrinkFactor):
        for obstacle in self.obstacles:
            obstacle.radius /= shrinkFactor 

    def addPatch(self, axes):
        for obstacle in self.obstacles:
            axes.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))

    def clearance(self, posX, posY):
        return 1

class Circle():
    """ Helper class for Circles representation"""
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def isValid(self, xy):
        x, y = xy
        if math.sqrt(pow(x - self.x, 2) + pow(y - self.y, 2)) - self.radius <= 0:
            return False
        return True
    

def getObstacles(ships, position, speedKmph, referenceLatlon):
    obstacle_type = rospy.get_param('obstacle_type', 'ellipse')
    obstacles = []
    if obstacle_type == "ellipse":
        for ship in ships:
            obstacles.append(Ellipse(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "wedge":
        for ship in ships:
            obstacles.append(Wedge(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "circles":
        for ship in ships:
            obstacles.append(Circles(ship, position, speedKmph, referenceLatlon))
    return obstacles

def pathCostThresholdExceeded(optimizationObjective, solutionPathObject):
    return solutionPathObject.cost(optimizationObjective).value() > COST_THRESHOLD

def pathDoesNotReachGoal(localPathLatlons, goal):
    lastWaypoint = (localPathLatlons[len(localPathLatlons) - 1].lat, localPathLatlons[len(localPathLatlons) - 1].lon)
    goal = (goal.lat, goal.lon)
    return distance(lastWaypoint, goal).kilometers > MAX_ALLOWABLE_DISTANCE_FINAL_WAYPOINT_TO_GOAL_KM
