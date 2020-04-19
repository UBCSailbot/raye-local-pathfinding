#!/usr/bin/env python
from datetime import datetime
from datetime import date
import pyautogui
import os
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
import rospy
import time
import sys
from plotting import plot_path, plot_path_2
from updated_geometric_planner import plan, indexOfObstacleOnPath
import planner_helpers as ph
import math 
from geopy.distance import distance
import geopy.distance
from std_msgs.msg import Float64
from local_pathfinding.msg import latlon, AISShip, AISMsg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from Sailbot import BoatState

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
AIS_BOAT_LENGTH_KM = 1
AIS_BOAT_CIRCLE_SPACING_KM = AIS_BOAT_RADIUS_KM * 1.5  # Distance between circles that make up an AIS boat

# Upwind downwind detection
UPWIND_DOWNWIND_TIME_LIMIT_SECONDS = 1.5

# Constants for pathfinding updates
COST_THRESHOLD = 20000
MAX_ALLOWABLE_DISTANCE_FINAL_WAYPOINT_TO_GOAL_KM = GLOBAL_WAYPOINT_REACHED_RADIUS_KM / 2

# Constants for obstacle models
WEDGE_EXPAND_ANGLE_DEGREES = 10.0
OBSTACLE_MAX_TIME_TO_LOC_HOURS = 3  # Do not extend objects more than X hours distance

class OMPLPath:
    """ Class for storing an OMPL configuration, OMPL path, and the referenceLatlon

    Attributes:
        _ss (ompl.geometric._geometric.SimpleSetup): SimpleSetup object that contains information about the OMPLPath, including its state space, space information, and cost function used to generate the path.
        _solutionPath (ompl.geometric._geometric.PathGeometric): PathGeometric object that represents the path, in XY coordinates with respect to a referenceLatlon
        _referenceLatlon (local_pathfinding.msg._latlon.latlon): latlon object that is the reference point with which the path's XY coordinates is set.
    """
    def __init__(self, ss, solutionPath, referenceLatlon):
        self._ss = ss
        self._solutionPath = solutionPath
        self._referenceLatlon = referenceLatlon

    ## Simple getter methods
    def getReferenceLatlon(self):
        return self._referenceLatlon

    def getSolutionPath(self):
        return self._solutionPath

    def getLength(self):
        return len(self._solutionPath.getStates())

    def getStateSpace(self):
        return self._ss.getStateSpace()

    def getSpaceInformation(self):
        return self._ss.getSpaceInformation()

    ## Longer getter methods
    def getCost(self):
        return self._solutionPath.cost(self._ss.getOptimizationObjective()).value()

    def getPathCostBreakdownString(self):
        # Assumes balanced optimization objective
        strings = []
        optimizationObjective = self._ss.getOptimizationObjective()
        for i in range(optimizationObjective.getObjectiveCount()):
            objective = optimizationObjective.getObjective(i)
            weight = optimizationObjective.getObjectiveWeight(i)
            cost = self._solutionPath.cost(objective).value()
            strings.append("{}: Cost = {}. Weight = {}. Weighted Cost = {} |||| ".format(type(objective).__name__, cost, weight, cost * weight))
        strings.append("--------------------------------------------------- ")
        strings.append("{}: Total Cost = {}".format(type(optimizationObjective).__name__, self._solutionPath.cost(optimizationObjective).value()))

        output = ''.join(strings)
        return output

    ## Methods that modify the class
    def updateWindDirection(self, state):
        # Set wind direction for cost evaluation
        globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
        objective = self._ss.getOptimizationObjective()  # Assumes balanced objective

        for i in range(objective.getObjectiveCount()):
            if isinstance(objective.getObjective(i), ph.WindObjective):
                objective.getObjective(i).windDirectionDegrees = globalWindDirectionDegrees
                return

        rospy.logwarn("updateWindDirection() was unsuccessful. Wind direction was not updated")

    def updateObstacles(self, state):
        # Set the objects used to check which states in the space are valid
        obstacles = getObstacles(state.AISData.ships, state.position, state.speedKmph, self._referenceLatlon)
        validity_checker = ph.ValidityChecker(self._ss.getSpaceInformation(), obstacles)
        self._ss.setStateValidityChecker(validity_checker)

    def removePastWaypoints(self, state):
        """
        Removes waypoints that the boat has already passed.
        Note: Should only be called once when waypointReached() == True, not every loop, otherwise the localWaypointReached tangent line could get messed up.
        Additional reason: keepAfter() method implementation assumes all waypoints are equally spaced
        Algorithm:
        1. Find index of waypoint closest to state, call that closestWaypoint
        2. Check dist(waypoint before closestWaypoint, state) and dist(waypoint after closestWaypoint, state)
            * If closer to waypoint BEFORE closestWaypoint: remove all waypoints before closestWaypoint, EXCLUDING closestWaypoint
            * If closer to waypoint AFTER closestWaypoint: remove all waypoints before closestWaypoint, INCLUDING closestWaypoint
        This answers the question: should we remove the closestWaypoint? Eg. Are we past the closestWaypoint or behind it?
        Algorithm works when all waypoints have about the same distance apart. Doesn't always work when waypoints are not all equally close

        Let B = Boat and numbers be waypoint numbers
        Boat is always aiming for index = 1
        Will replace the boat with index 0 after operation

        EXAMPLE 1 when it works:
        ------------------------
        Before:
        0   B  1     2     3  (B is closest to 1. Thus it compares dist(0, B) and dist(2, B). Decides to keep 1.)

        KeepAfter:
            B  0     1     2

        After add in boat position as first position:
            0  1     2     3

        Example 2 when it works:
        ------------------------
        Before:
        0      1B    2     3 (B is closest to 1. Thus it compares dist(0, B) and dist(2, B). Decides to remove 1.)

        KeepAfter:
                B    0     1

        After add in boat position as first position:
                0    1     2

        Example 3 when it doesn't work:
        ------------------------
        Before:
        0 1B    2     3 (B is closest to 1. Thus it compares dist(0, B) and dist(2, B). Decides to keep 1.)

        KeepAfter:
          0B    1     2

        After add in boat position as first position:
          10    2     3 (Boat may be told to go backwards)

        Example 4 when it doesn't work:
        ------------------------
        Before:
        0 B   1     2     3 (B is closest to 0. For this edge case, it just keeps everything.)

        KeepAfter:
        0 B   1     2     3

        After add in boat position as first position:
        1 0   2     3     4 (Boat may be told to go backwards)

        Therefore, in this method, we will have an edge case check after the KeepAfter operation.
        If (0 waypoints are removed) OR (1 waypoint is removed AND dist(B, 1) < dist(0,1)), then dont add position B as starting waypoint.
        Else add position B as a starting waypoint.
        """

        # Get current position in xy coordinates
        x, y = latlonToXY(state.position, self._referenceLatlon)
        positionXY = self._ss.getSpaceInformation().allocState()
        positionXY.setXY(x, y)

        # Keep waypoints only after your positionXY
        lengthBefore = self._solutionPath.getStateCount()
        self._solutionPath.keepAfter(positionXY)
        lengthAfter = self._solutionPath.getStateCount()

        # Only add in boat position as waypoint if edge case is avoided (described above)
        boatCouldGoWrongDirection = self.getStateSpace().distance(positionXY, self._solutionPath.getState(1)) < self.getStateSpace().distance(self._solutionPath.getState(0), self._solutionPath.getState(1))
        edgeCase = (lengthBefore - lengthAfter == 0) or ((lengthBefore - lengthAfter == 1) and boatCouldGoWrongDirection)
        if not edgeCase:
            self._solutionPath.prepend(positionXY)

class Path:
    def __init__(self, omplPath):
        self._omplPath = omplPath
        self._latlons = self._getLatlonsFromOMPLPath(self._omplPath)
        self._nextWaypointIndex = 1  # Next waypoint index is always 1, as the boat should always be aiming for the next upcoming waypoint

    def _getLatlonsFromOMPLPath(self, omplPath):
        # Convert solution path (in km WRT reference) into list of latlons
        path = []
        solutionPathObject = omplPath.getSolutionPath()
        referenceLatlon = omplPath.getReferenceLatlon()
        for state in solutionPathObject.getStates():
            xy = (state.getX(), state.getY())
            path.append(XYToLatlon(xy, referenceLatlon))
        return path

    def getNextWaypoint(self):
        def _getNextWaypoint(path, pathIndex):
            # If path is empty, return (0, 0)
            if len(path) == 0:
                rospy.logwarn("Path is empty.")
                rospy.logwarn("Setting waypoint to be (0, 0).")
                return latlon(0, 0)

            # If index out of range, return last waypoint in path
            if pathIndex >= len(path):
                rospy.logwarn("Path index is out of range: index = {} len(path) = {}".format(pathIndex, len(path)))
                rospy.logwarn("Setting waypoint to be the last element of the path")
                pathIndex = len(path) - 1
                return path[pathIndex]

            # If index in range, return the correct waypoint
            else:
                return path[pathIndex]

        return _getNextWaypoint(self._latlons, self._nextWaypointIndex)

    def getNextWaypointIndex(self):
        return self._nextWaypointIndex

    def getLatlons(self):
        return self._latlons

    def getOMPLPath(self):
        return self._omplPath

    def reachedEnd(self):
        return len(self._latlons) <= self._nextWaypointIndex

    def reachesGoalLatlon(self, goalLatlon):
        lastWaypointLatlon = self._latlons[len(self._latlons) - 1]
        lastWaypoint = (lastWaypointLatlon.lat, lastWaypointLatlon.lon)
        goal = (goalLatlon.lat, goalLatlon.lon)
        return distance(lastWaypoint, goal).kilometers <= MAX_ALLOWABLE_DISTANCE_FINAL_WAYPOINT_TO_GOAL_KM

    def nextWaypointReached(self, positionLatlon):
        # Convert from latlons to XY
        refLatlon = self._omplPath.getReferenceLatlon()
        previousWaypointLatlon = self._latlons[self._nextWaypointIndex - 1]
        nextWaypointLatlon = self._latlons[self._nextWaypointIndex]

        positionX, positionY = latlonToXY(positionLatlon, refLatlon)
        previousWaypointX, previousWaypointY = latlonToXY(previousWaypointLatlon, refLatlon)
        nextWaypointX, nextWaypointY = latlonToXY(nextWaypointLatlon, refLatlon)

        # Handle edge cases where waypoints have the same x or y component
        isStartNorth = nextWaypointY < previousWaypointY 
        isStartEast = nextWaypointX < previousWaypointX
        if nextWaypointX == previousWaypointX:
            if isStartNorth:
                return positionY <= nextWaypointY
            else:
                return positionY >= nextWaypointY
        if nextWaypointY == previousWaypointY:
            if isStartEast:
                return positionX <= nextWaypointX
            else:
                return positionX >= nextWaypointX

        # Create line in form y = mx + b that is perpendicular to the line from previousWaypoint to nextWaypoint
        tangentSlope = (nextWaypointY - previousWaypointY) / (nextWaypointX - previousWaypointX)
        normalSlope = -1/tangentSlope

        if nextWaypointX > 0:
            b = nextWaypointY + normalSlope * -math.fabs(nextWaypointX)
        else:
            b = nextWaypointY + normalSlope * math.fabs(nextWaypointX)
        y = lambda x: normalSlope * x + b
        x = lambda y: (y - b) / normalSlope 

    #    plt.xlim(-20, 20)
    #    plt.ylim(-20, 20)
    #    plt.plot([0], [0], marker = 'o', markersize=10, color="black")
    #    plt.plot([positionX], [positionY], marker = 'o', markersize=10, color="blue")
    #    plt.plot([previousWaypointX], [previousWaypointY], marker = 'o', markersize=10, color="green")
    #    plt.plot([nextWaypointX], [nextWaypointY], marker="o", markersize=10, color="red")
    #    x_plot = np.linspace(-200, 200, 100)
    #    plt.plot(x_plot, y(x_plot), '-r')
    #    plt.show()

        # Check if the line has been crossed
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

    def getCost(self):
        return self._omplPath.getCost()

    def getReferenceLatlon(self):
        return self._omplPath.getReferenceLatlon()

    def getSolutionPath(self):
        return self._omplPath.getSolutionPath()

    def getLength(self):
        return self._omplPath.getLength()

    def getStateSpace(self):
        return self._omplPath.getStateSpace()

    def getSpaceInformation(self):
        return self._omplPath.getSpaceInformation()

    def getPathCostBreakdownString(self):
        return self._omplPath.getPathCostBreakdownString()

    def updateWindDirection(self, state):
        self._omplPath.updateWindDirection(state)

    def updateObstacles(self, state):
        self._omplPath.updateObstacles(state)

    def removePastWaypoints(self, state):
        self._omplPath.removePastWaypoints(state)
        self._latlons = self._getLatlonsFromOMPLPath(self._omplPath)

    def upwindOrDownwindOnPath(self, state, numLookAheadWaypoints=None, showWarnings=False):
        # Default behavior when numLookAheadWaypoints is not given OR bad input: set to max
        maxNumLookAheadWaypoints = self.getLength() - self.getNextWaypointIndex()
        if numLookAheadWaypoints is None or numLookAheadWaypoints > maxNumLookAheadWaypoints:
            numLookAheadWaypoints = maxNumLookAheadWaypoints

        # Calculate global wind from measured wind and boat state
        globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
    
        # Get relevant waypoints (boat position first, then the next numLookAheadWaypoints startin from nextLocalWaypoint onwards)
        relevantWaypoints = []
        relevantWaypoints.append(latlonToXY(state.position, self.getReferenceLatlon()))
        for waypointIndex in range(self.getNextWaypointIndex(), self.getNextWaypointIndex() + numLookAheadWaypoints):
            waypoint = self._omplPath.getSolutionPath().getState(waypointIndex)
            relevantWaypoints.append([waypoint.getX(), waypoint.getY()])
    
        # Check relevantWaypoints for upwind or downwind sailing
        upwindOrDownwind = False
        for waypointIndex in range(1, len(relevantWaypoints)):
            # Calculate required heading between waypoints
            waypoint = relevantWaypoints[waypointIndex]
            prevWaypoint = relevantWaypoints[waypointIndex - 1]
            requiredHeadingDegrees = math.degrees(math.atan2(waypoint[1] - prevWaypoint[1], waypoint[0] - prevWaypoint[0]))
    
            if ph.isDownwind(math.radians(globalWindDirectionDegrees), math.radians(requiredHeadingDegrees)):
                if showWarnings:
                    rospy.loginfo("Downwind sailing on path detected. globalWindDirectionDegrees: {}. requiredHeadingDegrees: {}. waypointIndex: {}".format(globalWindDirectionDegrees, requiredHeadingDegrees, waypointIndex))
                upwindOrDownwind = True
                break
    
            elif ph.isUpwind(math.radians(globalWindDirectionDegrees), math.radians(requiredHeadingDegrees)):
                if showWarnings:
                    rospy.loginfo("Upwind sailing on path detected. globalWindDirectionDegrees: {}. requiredHeadingDegrees: {}. waypointIndex: {}".format(globalWindDirectionDegrees, requiredHeadingDegrees, waypointIndex))
                upwindOrDownwind = True
                break
    
        # Set counter to 0 on first use
        try:
            firstTime = self.lastTimeNotUpwindOrDownwind is None
        except AttributeError:
            rospy.loginfo("Handling first time case in upwindOrDownwindOnPath()")
            self.lastTimeNotUpwindOrDownwind = time.time()
    
        # Reset last time not upwind/downwind
        if not upwindOrDownwind:
            self.lastTimeNotUpwindOrDownwind = time.time()
            return False
    
        # Return true only if upwindOrDownwind for enough time
        consecutiveUpwindOrDownwindTimeSeconds = time.time() - self.lastTimeNotUpwindOrDownwind
        if consecutiveUpwindOrDownwindTimeSeconds >= UPWIND_DOWNWIND_TIME_LIMIT_SECONDS:
            if showWarnings:
                rospy.logwarn("Upwind/downwind sailing detected for {} seconds consecutively, which is greater than the {} second limit. This officially counts as upwind/downwind".format(consecutiveUpwindOrDownwindTimeSeconds, UPWIND_DOWNWIND_TIME_LIMIT_SECONDS))
            self.lastTimeNotUpwindOrDownwind = time.time()
            return True
        else:
            if showWarnings:
                rospy.loginfo("Upwind/downwind sailing detected for only {} seconds consecutively, which is less than the {} second limit. This does not count as upwind/downwind sailing yet.".format(consecutiveUpwindOrDownwindTimeSeconds, UPWIND_DOWNWIND_TIME_LIMIT_SECONDS))
            return False
    
    def obstacleOnPath(self, state, numLookAheadWaypoints=None, showWarnings=False):
        # Check if path will hit objects
        positionXY = latlonToXY(state.position, self.getReferenceLatlon())
        obstacles = getObstacles(state.AISData.ships, state.position, state.speedKmph, self.getReferenceLatlon())
    
        # Ensure nextLocalWaypointIndex + numLookAheadWaypoints is in bounds
        '''
        Let path = [(0,0), (1,1), (2,2), (3,3), (4,4)], len(path) = 5

        If: self.getNextWaypointIndex() = 2
        maxNumLookAheadWaypoints = self.getLength() - self.getNextWaypointIndex() = 5 - 2 = 3

        If numLookAheadWaypoints = 2, then care about waypoints (2,2), (3,3)
        If numLookAheadWaypoints = 6, cuts it down to 3, then care about waypoints (2,2), (3,3), (4,4)
        '''
        # Default behavior when numLookAheadWaypoints is not given OR bad input: set to max
        maxNumLookAheadWaypoints = self.getLength() - self.getNextWaypointIndex()
        if numLookAheadWaypoints is None or numLookAheadWaypoints > maxNumLookAheadWaypoints:
            numLookAheadWaypoints = maxNumLookAheadWaypoints
    
        self.updateObstacles(state)
        waypointIndexWithObstacle = indexOfObstacleOnPath(positionXY, self.getNextWaypointIndex(), numLookAheadWaypoints, self._omplPath)
        if waypointIndexWithObstacle != -1:
            if showWarnings:
                rospy.logwarn("Obstacle on path. waypointIndexWithObstacle: {}".format(waypointIndexWithObstacle))
            return True
        return False


def takeScreenshot():
    # Set imagePath on first time
    try:
        firstTime = (takeScreenshot.imagePath is None)
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

def createPath(state, runtimeSeconds=1.0, numRuns=2, resetSpeedupDuringPlan=False, speedupBeforePlan=1.0, maxAllowableRuntimeSeconds=MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS):
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
    globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)

    # If start or goal is invalid, shrink objects and re-run
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

    amountShrinkedStart = shrinkObstaclesUntilValid(start, obstacles)
    amountShrinkedGoal = shrinkObstaclesUntilValid(goal, obstacles)
    amountShrinked = max(amountShrinkedStart, amountShrinkedGoal)
    if amountShrinked > 1.0000001:
        rospy.logerr("Obstacles have been shrinked by factor of at most {}".format(amountShrinked))

    # Run the planner multiple times and find the best one
    rospy.loginfo("Running createLocalPathSS. runtimeSeconds: {}. numRuns: {}. Total time: {} seconds".format(runtimeSeconds, numRuns, runtimeSeconds*numRuns))

    # Create non-blocking plot showing the setup of the pathfinding problem. Useful to understand if the pathfinding problem is invalid or impossible
    shouldPlot = rospy.get_param('plot_pathfinding_problem', False)
    if shouldPlot:
        plotPathfindingProblem(globalWindDirectionDegrees, dimensions, start, goal, obstacles, state.headingDegrees, amountShrinked)

    # Take screenshot
    shouldTakeScreenshot = rospy.get_param('screenshot', False)
    if shouldTakeScreenshot:
        takeScreenshot()

    def isValidSolution(solution, referenceLatlon, state):
        if not solution.haveExactSolutionPath():
            return False
        return True

    validSolutions = []
    invalidSolutions = []
    for i in range(numRuns):
        # TODO: Incorporate globalWindSpeed into pathfinding?
        rospy.loginfo("Starting path-planning run number: {}".format(i))
        solution = plan(runtimeSeconds, "RRTStar", 'WeightedLengthAndClearanceCombo', globalWindDirectionDegrees, dimensions, start, goal, obstacles)
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
            rospy.logwarn("No valid solution can be found in under {} seconds. Using invalid solution.".format(maxAllowableRuntimeSeconds))
            break

        rospy.logwarn("Attempting to rerun with longer runtime: {} seconds".format(runtimeSeconds))
        solution = plan(runtimeSeconds, "RRTStar", 'WeightedLengthAndClearanceCombo', globalWindDirectionDegrees, dimensions, start, goal, obstacles)

        if isValidSolution(solution, referenceLatlon, state):
            validSolutions.append(solution)
        else:
            invalidSolutions.append(solution)

    def setAverageDistanceBetweenWaypoints(solutionPath):
        # Set the average distance between waypoints
        localPathLengthKm = solutionPath.length()
        numberOfLocalWaypoints = int(localPathLengthKm / AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM)
        solutionPath.interpolate(numberOfLocalWaypoints)

    # If no valid solutions found, use the best invalid one. Do not perform any path simplifying on invalid paths.
    if len(validSolutions) == 0:
        # Set the average distance between waypoints
        for solution in invalidSolutions:
            setAverageDistanceBetweenWaypoints(solution.getSolutionPath())

        bestSolution = min(invalidSolutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())
        bestSolutionPath = bestSolution.getSolutionPath()
        minCost = bestSolutionPath.cost(bestSolution.getOptimizationObjective()).value()
    else:
        # Set the average distance between waypoints
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

    plt.close()

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
    return distance(sailbot, waypt).kilometers < GLOBAL_WAYPOINT_REACHED_RADIUS_KM

def timeLimitExceeded(lastTimePathCreated, speedup):
    # TODO: Figure out a way to make changing speedup work properly for this
    # Shorter time limit when there is speedup
    pathUpdateTimeLimitSecondsSpeedup = PATH_UPDATE_TIME_LIMIT_SECONDS / speedup
    if time.time() - lastTimePathCreated > pathUpdateTimeLimitSecondsSpeedup:
        rospy.logwarn("{} seconds elapsed. Time limit of {} seconds was exceeded.".format(pathUpdateTimeLimitSecondsSpeedup, PATH_UPDATE_TIME_LIMIT_SECONDS))
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

    def clearance(self, xy):
        """ Return distance from obstacle to xy"""
        pass

    def shrink(self, shrinkFactor):
        """ Shrinks the obstacle by the shrink factor"""
        pass

class Ellipse():
    def __init__(self, x, y, height, width, angle):
        self.x = x
        self.y = y
        self.height = height
        self.width = width
        self.angle = angle
    
    def __str__(self):
        return str((self.x, self.y, self.height, self.width, self.angle))

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

class EllipseObstacle(ObstacleInterface, Ellipse):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
        self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)

    def __str__(self):
        return Ellipse.__str__(self)

    def _extendObstacle(self, aisData, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        aisX, aisY = latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)

        # Calculate length to extend boat
        distanceToBoatKm = distance((aisData.lat, aisData.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
        if sailbotSpeedKmph == 0 or distanceToBoatKm / sailbotSpeedKmph > OBSTACLE_MAX_TIME_TO_LOC_HOURS:
            timeToLocHours = OBSTACLE_MAX_TIME_TO_LOC_HOURS
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
         return Ellipse.isValid(self, xy)
        
    def _ellipseFormula(self, t): 
         return Ellipse._ellipseFormula(self, t)

    def addPatch(self, axes):
        Ellipse.addPatch(self, axes)

    def shrink(self, shrinkFactor):
        Ellipse.shrink(self, shrinkFactor)

    def clearance(self, xy):
        # TODO: Make this clearance better
        return (self.x - xy[0])**2 + (self.y - xy[1])**2

class Wedge(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
            ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
            self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)

    def __str__(self):
            return str((self.x, self.y, self.radius, self.theta1, self.theta2))
    
    def _extendObstacle(self, aisData, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        aisX, aisY = latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)

        theta1 = aisData.headingDegrees - WEDGE_EXPAND_ANGLE_DEGREES / 2.0
        theta2 = aisData.headingDegrees + WEDGE_EXPAND_ANGLE_DEGREES / 2.0
        if theta1 < 0:
            theta1 += 360
        if theta2 > 360:
            theta2 -= 360

        distanceToBoatKm = distance((aisData.lat, aisData.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
        if sailbotSpeedKmph == 0 or distanceToBoatKm / sailbotSpeedKmph > OBSTACLE_MAX_TIME_TO_LOC_HOURS:
            timeToLocHours = OBSTACLE_MAX_TIME_TO_LOC_HOURS
        else:
            timeToLocHours = distanceToBoatKm / sailbotSpeedKmph

        radius = aisData.speedKmph * timeToLocHours
        self.x, self.y = aisX, aisY
        self.radius = radius
        self.theta1, self.theta2 = theta1, theta2

    def addPatch(self, axes):
        axes.add_patch(patches.Wedge((self.x, self.y), self.radius, self.theta1, self.theta2))

    def isValid(self, xy):
        angle = math.degrees(math.atan2(xy[1] - self.y, xy[0] - self.x))
        if angle < 0:
            angle += 360
        distance = math.sqrt((xy[1] - self.y) **2 + (xy[0] - self.x) ** 2)
        if (angle > self.theta1 and angle < self.theta2 and distance < self.radius):
            return False
        return True

    def clearance(self, xy):
        # TODO: Make this clearance better
        return (self.x - xy[0])**2 + (self.y - xy[1])**2

    def shrink(self, shrinkFactor):
        self.radius /= shrinkFactor 

class Circles(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
            ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
            self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)

    def isValid(self, xy):
        for obstacle in self.obstacles:
            if not obstacle.isValid(xy):
                return False
        return True

    def _extendObstacle(self, aisData, sailbotPosition, sailbotSpeedKmph, referenceLatlon):
        self.obstacles = []
        aisX, aisY = latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)
        # Calculate length to extend boat
        distanceToBoatKm = distance((aisData.lat, aisData.lon), (sailbotPosition.lat, sailbotPosition.lon)).kilometers
        if sailbotSpeedKmph == 0 or distanceToBoatKm / sailbotSpeedKmph > OBSTACLE_MAX_TIME_TO_LOC_HOURS:
            timeToLocHours = OBSTACLE_MAX_TIME_TO_LOC_HOURS
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
            obstacle.shrink(shrinkFactor)

    def addPatch(self, axes):
        for obstacle in self.obstacles:
            obstacle.addPatch(axes)

    def clearance(self, xy):
        # TODO: Make this clearance better
        return (self.obstacles[0].x - xy[0])**2 + (self.obstacles[0].y - xy[1])**2

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

    def addPatch(self, axes):
        axes.add_patch(plt.Circle((self.x, self.y), radius=self.radius))

    def shrink(self, shrinkFactor):
        self.radius /= shrinkFactor 

class HybridEllipse(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
        self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)
        self.xy = latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)
        self.ellipse = Ellipse(self.xy[0], self.xy[1], AIS_BOAT_RADIUS_KM, AIS_BOAT_LENGTH_KM, aisData.headingDegrees)

    def __str__(self):
        return str(self.ellipse) + str(self.wedge)

    def _extendObstacle(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        self.wedge = Wedge(aisData, sailbotPosition, speedKmph, referenceLatlon)

    def addPatch(self, axes):
        self.wedge.addPatch(axes)
        self.ellipse.addPatch(axes)

    def isValid(self, xy):
        return (self.wedge.isValid(xy) and self.ellipse.isValid(xy))        

    def clearance(self, xy):
        return (self.xy[0] - xy[0])**2 + (self.xy[1] - xy[1])**2

    def shrink(self, shrinkFactor):
        self.ellipse.shrink(shrinkFactor)
        self.wedge.shrink(shrinkFactor)
    
class HybridCircle(ObstacleInterface):
    def __init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        ObstacleInterface.__init__(self, aisData, sailbotPosition, speedKmph, referenceLatlon)
        self._extendObstacle(self.aisData, self.sailbotPosition, self.speedKmph, self.referenceLatlon)
        self.xy = latlonToXY(latlon(aisData.lat, aisData.lon), referenceLatlon)
        self.circle = Circle(self.xy[0], self.xy[1], AIS_BOAT_RADIUS_KM)

    def __str__(self):
        return str(self.circle) + str(self.wedge)

    def _extendObstacle(self, aisData, sailbotPosition, speedKmph, referenceLatlon):
        self.wedge = Wedge(aisData, sailbotPosition, speedKmph, referenceLatlon)

    def addPatch(self, axes):
        self.wedge.addPatch(axes)
        self.circle.addPatch(axes)

    def isValid(self, xy):
        return (self.wedge.isValid(xy) and self.circle.isValid(xy))        

    def clearance(self, xy):
        return (self.xy[0] - xy[0])**2 + (self.xy[1] - xy[1])**2

    def shrink(self, shrinkFactor):
        self.circle.shrink(shrinkFactor)
        self.wedge.shrink(shrinkFactor)

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
