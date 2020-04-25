#!/usr/bin/env python
from datetime import datetime
from datetime import date
import os
from ompl import util as ou
from ompl import geometric as og
import rospy
import time
import sys
from updated_geometric_planner import plan
import math
from geopy.distance import distance
from std_msgs.msg import Float64
from local_pathfinding.msg import latlon
import matplotlib.pyplot as plt
from matplotlib import patches
from Path import Path, OMPLPath
from obstacles import EllipseObstacle, Wedge, Circles, HybridEllipse, HybridCircle

# Location constants
PORT_RENFREW_LATLON = latlon(48.5, -124.8)
MAUI_LATLON = latlon(20.0, -156.0)

# Constants
AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM = 3.0
GLOBAL_WAYPOINT_REACHED_RADIUS_KM = 10.0
PATH_UPDATE_TIME_LIMIT_SECONDS = 7200

# Pathfinding constants
MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS = 20.0
INCREASE_RUNTIME_FACTOR = 1.5
OBSTACLE_SHRINK_FACTOR = 1.2

# Scale NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES and NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND to change based on
# waypoint distance
LOOK_AHEAD_FOR_OBSTACLES_KM = 20
NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES = int(math.ceil(LOOK_AHEAD_FOR_OBSTACLES_KM /
                                                       AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM))
LOOK_AHEAD_FOR_UPWIND_DOWNWIND_KM = 10
NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND = int(math.ceil(LOOK_AHEAD_FOR_UPWIND_DOWNWIND_KM /
                                                             AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM))

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

# Constants for pathfinding updates
COST_THRESHOLD = 20000


def takeScreenshot():
    # Put import here to avoid CI issues with pyautogui
    import pyautogui

    # Set imagePath on first time
    try:
        takeScreenshot.imagePath
    except AttributeError:
        rospy.loginfo("Handling first time case in takeScreenshot()")
        pathToThisFile = os.path.dirname(os.path.abspath(__file__))
        dateString = date.today().strftime("%b-%d-%Y")
        timeString = datetime.now().strftime('%H-%M-%S')
        pathToDateFolder = "{}/../images/{}".format(pathToThisFile, dateString)
        pathToStartTimeFolder = "{}/{}".format(pathToDateFolder, timeString)
        if not os.path.exists(pathToDateFolder):
            os.mkdir(pathToDateFolder)
        if not os.path.exists(pathToStartTimeFolder):
            os.mkdir(pathToStartTimeFolder)
        takeScreenshot.imagePath = pathToStartTimeFolder

    # Take screenshot
    time.sleep(1)
    myScreenshot = pyautogui.screenshot()

    # Save screenshot
    timeString = datetime.now().strftime('%H-%M-%S')
    fullImagePath = "{}/{}.png".format(takeScreenshot.imagePath, timeString)
    rospy.loginfo("Taking screenshot...")
    myScreenshot.save(fullImagePath)
    rospy.loginfo("Screenshot saved to {}".format(fullImagePath))


def latlonToXY(latlon, referenceLatlon):
    x = distance((referenceLatlon.lat, referenceLatlon.lon), (referenceLatlon.lat, latlon.lon)).kilometers
    y = distance((referenceLatlon.lat, referenceLatlon.lon), (latlon.lat, referenceLatlon.lon)).kilometers
    if referenceLatlon.lon > latlon.lon:
        x = -x
    if referenceLatlon.lat > latlon.lat:
        y = -y
    return [x, y]


def XYToLatlon(xy, referenceLatlon):
    x_distance = distance(kilometers=xy[0])
    y_distance = distance(kilometers=xy[1])
    destination = x_distance.destination(point=(referenceLatlon.lat, referenceLatlon.lon), bearing=BEARING_EAST)
    destination = y_distance.destination(point=(destination.latitude, destination.longitude), bearing=BEARING_NORTH)
    return latlon(destination.latitude, destination.longitude)


def plotPathfindingProblem(globalWindDirectionDegrees, dimensions, start, goal, obstacles, headingDegrees,
                           amountObstaclesShrinked):
    # Clear plot if already there
    plt.cla()

    # Create plot with start and goal
    x_min, y_min, x_max, y_max = dimensions
    markersize = min(x_max - x_min, y_max - y_min) / 2
    plt.ion()
    axes = plt.gca()
    goalPlot, = axes.plot(goal[0], goal[1], marker='*', color='y',
                          markersize=markersize)                          # Yellow star
    # Red triangle with correct heading. The (-90) is because the triangle
    # default heading 0 points North, but this heading has 0 be East.
    startPlot, = axes.plot(start[0], start[1], marker=(3, 0, headingDegrees - 90), color='r', markersize=markersize)

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

    arrowLength = min(dimensions[2] - dimensions[0], dimensions[3] - dimensions[1]) / 15
    arrowCenter = (dimensions[0] + 1.5 * arrowLength, dimensions[3] - 1.5 * arrowLength)
    arrowStart = (arrowCenter[0] - 0.5 * arrowLength * math.cos(math.radians(globalWindDirectionDegrees)),
                  arrowCenter[1] - 0.5 * arrowLength * math.sin(math.radians(globalWindDirectionDegrees)))
    windDirection = patches.FancyArrow(arrowStart[0], arrowStart[1],
                                       arrowLength * math.cos(math.radians(globalWindDirectionDegrees)),
                                       arrowLength * math.sin(math.radians(globalWindDirectionDegrees)),
                                       width=arrowLength / 4)
    axes.add_patch(windDirection)

    # Draw plot
    plt.draw()
    plt.pause(0.001)


def createPath(state, runtimeSeconds=1.0, numRuns=2, resetSpeedupDuringPlan=False, speedupBeforePlan=1.0,
               maxAllowableRuntimeSeconds=MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS):
    # Helper methods
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

    def shrinkObstaclesUntilValid(xy, obstacles):
        def obstaclesTooClose(xy, obstacles):
            obstaclesTooCloseList = []
            for obstacle in obstacles:
                if not obstacle.isValid(xy):
                    obstaclesTooCloseList.append(obstacle)
            return obstaclesTooCloseList
        amountShrinked = 1.0
        obstaclesTooCloseList = obstaclesTooClose(xy, obstacles)
        while len(obstaclesTooCloseList) > 0:
            rospy.logerr("start or goal state is not valid")
            rospy.logerr("Shrinking some obstacles by a factor of {}".format(OBSTACLE_SHRINK_FACTOR))
            for o in obstaclesTooCloseList:
                o.shrink(OBSTACLE_SHRINK_FACTOR)
            obstaclesTooCloseList = obstaclesTooClose(xy, obstacles)
            amountShrinked *= OBSTACLE_SHRINK_FACTOR
        return amountShrinked
    ou.setLogLevel(ou.LOG_WARN)

    def isValidSolution(solution, referenceLatlon, state):
        if not solution.haveExactSolutionPath():
            return False
        return True

    def setAverageDistanceBetweenWaypoints(solutionPath):
        # Set the average distance between waypoints
        localPathLengthKm = solutionPath.length()
        numberOfLocalWaypoints = int(localPathLengthKm / AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM)
        solutionPath.interpolate(numberOfLocalWaypoints)

    def findBestSolution(validSolutions, invalidSolutions):
        # If no valid solutions found, use the best invalid one. Do not perform any path simplifying on invalid paths.
        if len(validSolutions) == 0:
            # Set the average distance between waypoints. Must be done before cost calculation and comparison
            for solution in invalidSolutions:
                setAverageDistanceBetweenWaypoints(solution.getSolutionPath())

            bestSolution = min(invalidSolutions,
                               key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())
            bestSolutionPath = bestSolution.getSolutionPath()
            minCost = bestSolutionPath.cost(bestSolution.getOptimizationObjective()).value()
        else:
            # Set the average distance between waypoints. Must be done before cost calculation and comparison
            for solution in validSolutions:
                setAverageDistanceBetweenWaypoints(solution.getSolutionPath())

            # Find path with minimum cost. Can be either simplified or unsimplified path.
            # Need to recheck that simplified paths are valid before using
            minCost = sys.maxsize
            bestSolution = None
            bestSolutionPath = None
            for solution in validSolutions:
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
                setAverageDistanceBetweenWaypoints(simplifiedPath)
                simplifiedCost = simplifiedPath.cost(solution.getOptimizationObjective()).value()
                if simplifiedCost < minCost:
                    # Double check that simplified path is valid
                    simplifiedPathObject = Path(OMPLPath(solution, simplifiedPath, referenceLatlon))
                    hasObstacleOnPath = simplifiedPathObject.obstacleOnPath(state)
                    hasUpwindOrDownwindOnPath = simplifiedPathObject.upwindOrDownwindOnPath(state)
                    isStillValid = not hasObstacleOnPath and not hasUpwindOrDownwindOnPath
                    if isStillValid:
                        bestSolution = solution
                        bestSolutionPath = simplifiedPath
                        minCost = simplifiedCost
        return bestSolution, bestSolutionPath, minCost

    # Set speedup to 1.0 during planning
    if resetSpeedupDuringPlan:
        speedupDuringPlan = 1.0
        rospy.loginfo("Setting speedup to this value during planning = {}".format(speedupDuringPlan))
        publisher = rospy.Publisher('speedup', Float64, queue_size=4)
        publisher.publish(speedupDuringPlan)

    # Get setup parameters from state for ompl plan()
    # Convert all latlons to NE in km wrt referenceLatlon
    referenceLatlon = state.globalWaypoint
    start = latlonToXY(state.position, referenceLatlon)
    goal = latlonToXY(state.globalWaypoint, referenceLatlon)
    dimensions = getXYLimits(start, goal)
    obstacles = getObstacles(state.AISData.ships, state.position, state.speedKmph, referenceLatlon)
    globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(
        state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)

    # If start or goal is invalid, shrink objects and re-run
    amountShrinkedStart = shrinkObstaclesUntilValid(start, obstacles)
    amountShrinkedGoal = shrinkObstaclesUntilValid(goal, obstacles)
    amountShrinked = max(amountShrinkedStart, amountShrinkedGoal)
    if amountShrinked > 1.0000001:
        rospy.logerr("Obstacles have been shrinked by factor of at most {}".format(amountShrinked))

    # Run the planner multiple times and find the best one
    rospy.loginfo("Running createLocalPathSS. runtimeSeconds: {}. numRuns: {}. Total time: {} seconds"
                  .format(runtimeSeconds, numRuns, runtimeSeconds * numRuns))

    # Create non-blocking plot showing the setup of the pathfinding problem.
    # Useful to understand if the pathfinding problem is invalid or impossible
    shouldPlot = rospy.get_param('plot_pathfinding_problem', False)
    if shouldPlot:
        plotPathfindingProblem(globalWindDirectionDegrees, dimensions, start, goal, obstacles,
                               state.headingDegrees, amountShrinked)

    # Take screenshot
    shouldTakeScreenshot = rospy.get_param('screenshot', False)
    if shouldTakeScreenshot:
        takeScreenshot()

    # Look for solutions
    validSolutions = []
    invalidSolutions = []
    for i in range(numRuns):
        rospy.loginfo("Starting path-planning run number: {}".format(i))
        solution = plan(runtimeSeconds, "RRTStar", 'WeightedLengthAndClearanceCombo',
                        globalWindDirectionDegrees, dimensions, start, goal, obstacles)
        if isValidSolution(solution, referenceLatlon, state):
            validSolutions.append(solution)
        else:
            invalidSolutions.append(solution)

    # If no validSolutions found, re-run with larger runtime
    totalRuntimeSeconds = numRuns * runtimeSeconds
    while len(validSolutions) == 0:
        rospy.logwarn("No valid solutions found in {} seconds runtime".format(runtimeSeconds))
        runtimeSeconds *= INCREASE_RUNTIME_FACTOR
        totalRuntimeSeconds += runtimeSeconds

        # If valid solution can't be found for large runtime, then stop searching
        if totalRuntimeSeconds >= maxAllowableRuntimeSeconds:
            rospy.logwarn("No valid solution can be found in under {} seconds. Using invalid solution."
                          .format(maxAllowableRuntimeSeconds))
            break

        rospy.logwarn("Attempting to rerun with longer runtime: {} seconds".format(runtimeSeconds))
        solution = plan(runtimeSeconds, "RRTStar", 'WeightedLengthAndClearanceCombo',
                        globalWindDirectionDegrees, dimensions, start, goal, obstacles)

        if isValidSolution(solution, referenceLatlon, state):
            validSolutions.append(solution)
        else:
            invalidSolutions.append(solution)

    # Find best solution
    bestSolution, bestSolutionPath, minCost = findBestSolution(validSolutions, invalidSolutions)

    # Close plot if it was started
    plt.close()

    # Reset speedup back to original value
    if resetSpeedupDuringPlan:
        rospy.loginfo("Setting speedup back to its value before planning = {}".format(speedupBeforePlan))
        publisher = rospy.Publisher('speedup', Float64, queue_size=4)
        publisher.publish(speedupBeforePlan)

    return Path(OMPLPath(bestSolution, bestSolutionPath, referenceLatlon))


def globalWaypointReached(position, globalWaypoint):
    # TODO: Consider fixing globalWaypointReached to not go backwards
    sailbot = (position.lat, position.lon)
    waypt = (globalWaypoint.lat, globalWaypoint.lon)
    dist = distance(sailbot, waypt).kilometers
    return dist < GLOBAL_WAYPOINT_REACHED_RADIUS_KM


def timeLimitExceeded(lastTimePathCreated, speedup):
    # TODO: Figure out a way to make changing speedup work properly for this
    # Shorter time limit when there is speedup
    pathUpdateTimeLimitSecondsSpeedup = PATH_UPDATE_TIME_LIMIT_SECONDS / speedup
    if time.time() - lastTimePathCreated > pathUpdateTimeLimitSecondsSpeedup:
        rospy.logwarn("{} seconds elapsed. Time limit of {} seconds was exceeded.".format(
            pathUpdateTimeLimitSecondsSpeedup, PATH_UPDATE_TIME_LIMIT_SECONDS))
        return True
    else:
        return False


def getDesiredHeadingDegrees(position, localWaypoint):
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
    trueWindSpeedXGlobalFrame = trueWindSpeedXBoatFrame * \
        math.sin(math.radians(headingDegrees)) + trueWindSpeedYBoatFrame * math.cos(math.radians(headingDegrees))
    trueWindSpeedYGlobalFrame = trueWindSpeedYBoatFrame * \
        math.sin(math.radians(headingDegrees)) - trueWindSpeedXBoatFrame * math.cos(math.radians(headingDegrees))

    # Calculate global wind speed and direction
    globalWindSpeed = (trueWindSpeedXGlobalFrame**2 + trueWindSpeedYGlobalFrame**2)**0.5
    globalWindDirectionDegrees = math.degrees(math.atan2(trueWindSpeedYGlobalFrame, trueWindSpeedXGlobalFrame))

    return globalWindSpeed, globalWindDirectionDegrees


def globalWindToMeasuredWind(globalWindSpeed, globalWindDirectionDegrees, boatSpeed, headingDegrees):
    # Calculate the measuredWindSpeed in the global frame
    measuredWindSpeedXGlobalFrame = globalWindSpeed * \
        math.cos(math.radians(globalWindDirectionDegrees)) - boatSpeed * math.cos(math.radians(headingDegrees))
    measuredWindSpeedYGlobalFrame = globalWindSpeed * \
        math.sin(math.radians(globalWindDirectionDegrees)) - boatSpeed * math.sin(math.radians(headingDegrees))

    # Calculated the measuredWindSpeed in the boat frame
    measuredWindSpeedXBoatFrame = measuredWindSpeedXGlobalFrame * \
        math.sin(math.radians(headingDegrees)) - measuredWindSpeedYGlobalFrame * math.cos(math.radians(headingDegrees))
    measuredWindSpeedYBoatFrame = measuredWindSpeedXGlobalFrame * \
        math.cos(math.radians(headingDegrees)) + measuredWindSpeedYGlobalFrame * math.sin(math.radians(headingDegrees))

    measuredWindDirectionDegrees = math.degrees(math.atan2(measuredWindSpeedYBoatFrame, measuredWindSpeedXBoatFrame))
    measuredWindSpeed = (measuredWindSpeedYBoatFrame**2 + measuredWindSpeedXBoatFrame**2)**0.5

    return measuredWindSpeed, measuredWindDirectionDegrees


def headingToBearingDegrees(headingDegrees):
    # Heading is defined using cartesian coordinates. 0 degrees is East. 90 degrees in North. 270 degrees is South.
    # Bearing is defined differently. 0 degrees is North. 90 degrees is East. 180 degrees is South.
    # Heading = -Bearing + 90
    return -headingDegrees + 90


def isValid(xy, obstacles):
    for obstacle in obstacles:
        if not obstacle.isValid(xy):
            return False
    return True


def getObstacles(ships, position, speedKmph, referenceLatlon):
    obstacle_type = rospy.get_param('obstacle_type', 'ellipse')
    obstacles = []
    if obstacle_type == "ellipse":
        for ship in ships:
            obstacles.append(EllipseObstacle(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "wedge":
        for ship in ships:
            obstacles.append(Wedge(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "circles":
        for ship in ships:
            obstacles.append(Circles(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "hybrid_ellipse":
        for ship in ships:
            obstacles.append(HybridEllipse(ship, position, speedKmph, referenceLatlon))
    elif obstacle_type == "hybrid_circle":
        for ship in ships:
            obstacles.append(HybridCircle(ship, position, speedKmph, referenceLatlon))
    return obstacles


def pathCostThresholdExceeded(currentCost):
    return currentCost > COST_THRESHOLD


def waitForGlobalPath(sailbot):
    while not sailbot.newGlobalPathReceived:
        # Exit if shutdown
        if rospy.is_shutdown():
            rospy.loginfo("rospy.is_shutdown() is True. Exiting")
            sys.exit()
        else:
            rospy.loginfo("Waiting for sailbot to receive first newGlobalPath")
            time.sleep(1)
    rospy.loginfo("newGlobalPath received")
