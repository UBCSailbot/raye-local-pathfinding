import math
import sys
import time
import rospy
from updated_geometric_planner import indexOfObstacleOnPath
import planner_helpers as ph
import utilities as utils
import obstacles as obs
from sailbot_msg.msg import latlon
from geopy.distance import distance
import matplotlib.pyplot as plt
from matplotlib import patches
from ompl import util as ou
from updated_geometric_planner import plan
from ompl import geometric as og
from std_msgs.msg import Float64

# Constants
UPWIND_DOWNWIND_TIME_LIMIT_SECONDS = 2.0
MAX_ALLOWABLE_DISTANCE_FINAL_WAYPOINT_TO_GOAL_KM = 5

# Pathfinding constants
MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS = 20.0
INCREASE_RUNTIME_FACTOR = 1.5
WAYPOINT_REACHED_DISTANCE = 0.5
GLOBAL_WAYPOINT_REACHED_DISTANCE = 0.5

# Global variables to count invalid solutions
count_invalid_solutions = 0
temp_invalid_solutions = 0


def getPerpLine(lastXY, nextXY, isGlobal=False):
    '''
    Returns:
        bool isStartNorth, bool isStartEast, slope, y-intercept
            - a vertical line, with undefined slope and no y-intercept, is represented by
                slope, y-intercept = None, None
    '''
    lastX, lastY = lastXY
    nextX, nextY = nextXY
    dist = GLOBAL_WAYPOINT_REACHED_DISTANCE if isGlobal else WAYPOINT_REACHED_DISTANCE

    isStartNorth = nextY < lastY
    isStartEast = nextX < lastX
    if abs(nextX - lastX) < 0.01:
        if isStartNorth:
            return isStartNorth, isStartEast, 0, nextY + dist
        else:
            return isStartNorth, isStartEast, 0, nextY - dist

    if abs(nextY - lastY) < 0.01:
        if isStartEast:
            return isStartNorth, isStartEast, None, None
        else:
            return isStartNorth, isStartEast, None, None

    # Create line in form y = mx + b that is perpendicular to the line from previousWaypoint to nextWaypoint
    tangentSlope = (nextY - lastY) / (nextX - lastX)
    normalSlope = -1 / tangentSlope
    b = nextY - normalSlope * nextX

    # Modify y-intercept so that distance between the original and shifted lines has a
    # magnitude of dist in direction of previousWaypoint.
    # NOte that cos(arctan(x)) can never be 0, so no division by 0 will happen
    verticalShift = dist / math.cos(math.atan(math.fabs(normalSlope)))
    if isStartNorth:
        b += verticalShift
    else:
        b -= verticalShift

    return isStartNorth, isStartEast, normalSlope, b


class OMPLPath:
    """ Class for storing an OMPL configuration, OMPL path, and the referenceLatlon

    Attributes:
        _ss (ompl.geometric._geometric.SimpleSetup): SimpleSetup object that contains information about the OMPLPath,
                                                     including its state space, space information, and cost function
                                                     used to generate the path.
        _solutionPath (ompl.geometric._geometric.PathGeometric): PathGeometric object that represents the path, in XY
                                                                 coordinates with respect to a referenceLatlon.
        _referenceLatlon (sailbot_msg.msg._latlon.latlon): latlon object that is the reference point with which
                                                                 the path's XY coordinates is set.
    """

    def __init__(self, ss, solutionPath, referenceLatlon):
        self._ss = ss
        self._solutionPath = solutionPath
        self._referenceLatlon = referenceLatlon

    # Simple getter methods
    def getSS(self):
        return self._ss

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

    # Longer getter methods
    def getCost(self):
        return self._solutionPath.cost(self._ss.getOptimizationObjective()).value()

    def getObstacles(self, state):
        self.updateObstacles(state)
        return self._obstacles

    def getPathCostBreakdownString(self):
        '''Return a string showing the breakdown of how the path cost was calculated.'''
        # Assumes balanced optimization objective
        strings = []
        optimizationObjective = self._ss.getOptimizationObjective()
        for i in range(optimizationObjective.getObjectiveCount()):
            objective = optimizationObjective.getObjective(i)
            weight = optimizationObjective.getObjectiveWeight(i)
            cost = self._solutionPath.cost(objective).value()
            strings.append("{}: Cost = {}. Weight = {}. Weighted Cost = {} |||| "
                           .format(type(objective).__name__, cost, weight, cost * weight))
        strings.append("--------------------------------------------------- ")
        strings.append("{}: Total Cost = {}".format(type(optimizationObjective).__name__,
                                                    self._solutionPath.cost(optimizationObjective).value()))

        output = ''.join(strings)
        return output

    # Methods that modify the class
    def updateWindDirection(self, state):
        '''Change the wind direction of the OMPL objective function, which changes subsequent path cost calculations.'''
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
        objective = self._ss.getOptimizationObjective()  # Assumes balanced objective

        for i in range(objective.getObjectiveCount()):
            if isinstance(objective.getObjective(i), ph.WindObjective):
                objective.getObjective(i).windDirectionDegrees = globalWindDirectionDegrees
                return

        rospy.logwarn("updateWindDirection() was unsuccessful. Wind direction was not updated")

    def updateObstacles(self, state):
        '''Update the obstacles of the OMPL validation checker'''
        self._obstacles = obs.getObstacles(state, self._referenceLatlon)
        validity_checker = ph.ValidityChecker(self._ss.getSpaceInformation(), self._obstacles)
        self._ss.setStateValidityChecker(validity_checker)

    def removePastWaypoints(self, state):
        """Removes waypoints that the boat has already passed.

        Note: Best used when waypointReached() == True, not every loop
        Reason: keepAfter() method implementation assumes all waypoints are equally spaced
        keepAfter() algorithm:
        1. Find index of waypoint closest to state, call that closestWaypoint
        2. Check dist(waypoint before closestWaypoint, state) and dist(waypoint after closestWaypoint, state)
            * If closer to waypoint BEFORE closestWaypoint: remove all waypoints before closestWaypoint
              EXCLUDING closestWaypoint
            * If closer to waypoint AFTER closestWaypoint: remove all waypoints before closestWaypoint
              INCLUDING closestWaypoint
        This answers the question: should we remove the closestWaypoint?
        Eg. Are we past the closestWaypoint or behind it?
        Algorithm works when all waypoints have about the same distance apart.
        Doesn't always work when waypoints are not all equally close.

        Let B = Boat and numbers be waypoint numbers
        Boat is always aiming for index = 1
        Will replace the boat with index 0 after operation

        Example 1 (regular behavior)
        ------------------------
        Before:
        0   B  1     2     3  (B is closest to 1. Thus it compares dist(0, B) and dist(2, B). Decides to keep 1.)

        KeepAfter:
            B  0     1     2

        Example 2 (cases where we overshoot the waypoint, so we skip a waypoint
        ------------------------
        Before:
        0      1B    2     3 (B is closest to 1. Thus it compares dist(0, B) and dist(2, B). Decides to remove 1.)

        KeepAfter:
                B    0     1

        Example 3 (edge case where the boat's position must be prepended to the path to avoid out of bounds)
        ------------------------
        Before:
        0     1     2 B   3 (B is closest to 2. Thus it compares dist(1, B) and dist(3, B). Decides to remove 2.)

        KeepAfter:
                      B   0

        After add in boat position as first position:
                      0   1

        Example 4 (strange case with irregular spacing)
        ------------------------
        Before:
        0 1B    2     3 (B is closest to 1. Thus it compares dist(0, B) and dist(2, B). Decides to keep 1.)

        KeepAfter:
          0B    1     2

        Example 5
        ------------------------
        Before:
        0 B   1     2     3 (B is closest to 0. For this edge case, it just keeps everything.)

        KeepAfter:
        0 B   1     2     3

        Therefore, in this method, we will have an edge case check after the keepAfter operation.
        If lengthAfter == 1, then add position B as a starting waypoint.
        """

        # Get current position in xy coordinates
        x, y = utils.latlonToXY(state.position, self._referenceLatlon)
        positionXY = self._ss.getSpaceInformation().allocState()
        positionXY.setXY(x, y)

        # Keep waypoints only after your positionXY
        lengthBefore = self._solutionPath.getStateCount()
        self._solutionPath.keepAfter(positionXY)
        lengthAfter = self._solutionPath.getStateCount()
        rospy.loginfo("lengthBefore = {}. lengthAfter = {}".format(lengthBefore, lengthAfter))

        if lengthAfter == 0:
            raise RuntimeError("lengthAfter == 0, can't perform pathfinding. keepAfter() won't do this")
        if lengthAfter == 1:
            rospy.loginfo("lengthAfter == 1, prepending boat position to path so path length is at least 2")
            self._solutionPath.prepend(positionXY)


class Path:
    """ Class for storing and interacting with a path in both latlon and NE (xy) coordinates

    Attributes:
        _omplPath (OMPLPath): OMPLPath object represents the path in NE (xy) coordinates
        _latlons (list of latlons): List of latlons that corresponds with _omplPath's waypoints
        _nextWaypointIndex (int): index of the next waypoint in _latlons that the boat should be going towards
    """

    def __init__(self, omplPath):
        self._omplPath = omplPath
        self._latlons = self._getLatlonsFromOMPLPath(self._omplPath)

        # Next waypoint index is always 1, as the boat should always be aiming for the next upcoming waypoint
        self._nextWaypointIndex = 1

    def _getLatlonsFromOMPLPath(self, omplPath):
        '''Convert solution path (in km WRT reference) into list of latlons'''
        path = []
        solutionPathObject = omplPath.getSolutionPath()
        referenceLatlon = omplPath.getReferenceLatlon()
        for state in solutionPathObject.getStates():
            xy = (state.getX(), state.getY())
            path.append(utils.XYToLatlon(xy, referenceLatlon))
        return path

    # Simple methods
    def getNextWaypointIndex(self):
        return self._nextWaypointIndex

    def getLatlons(self):
        return self._latlons

    def getOMPLPath(self):
        return self._omplPath

    def reachedEnd(self):
        return len(self._latlons) <= self._nextWaypointIndex

    def getPreviousWaypoint(self):
        '''Gets the previous waypoint, but performs bounds and edge case checks'''
        def _getPreviousWaypoint(path, pathIndex):
            # If path is empty, return (0,0)
            if len(path) == 0:
                rospy.logwarn("Path is empty.")
                rospy.logwarn("Setting previous waypoint to be (0, 0).")
                return latlon(0, 0)

            # If index out of range, return last waypoint in path
            if pathIndex >= len(path):
                rospy.logwarn("Path index is out of range: index = {} len(path) = {}".format(pathIndex, len(path)))
                rospy.logwarn("Setting previous waypoint to be the second to last element of the path")
                pathIndex = len(path) - 2
                return path[pathIndex]

            # If index in range, return the correct waypoint
            else:
                return path[pathIndex]

        return _getPreviousWaypoint(self._latlons, self._nextWaypointIndex - 1)

    def getNextWaypoint(self):
        '''Gets the next waypoint, but performs bounds and edge case checks'''
        def _getNextWaypoint(path, pathIndex):
            # If path is empty, return (0, 0)
            if len(path) == 0:
                rospy.logwarn("Path is empty.")
                rospy.logwarn("Setting next waypoint to be (0, 0).")
                return latlon(0, 0)

            # If index out of range, return last waypoint in path
            if pathIndex >= len(path):
                rospy.logwarn("Path index is out of range: index = {} len(path) = {}".format(pathIndex, len(path)))
                rospy.logwarn("Setting next waypoint to be the last element of the path")
                pathIndex = len(path) - 1
                return path[pathIndex]

            # If index in range, return the correct waypoint
            else:
                return path[pathIndex]

        return _getNextWaypoint(self._latlons, self._nextWaypointIndex)

    # Simple methods that directly call _omplPath methods
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

    def getObstacles(self, state):
        return self._omplPath.getObstacles(state)

    def updateWindDirection(self, state):
        self._omplPath.updateWindDirection(state)

    def updateObstacles(self, state):
        self._omplPath.updateObstacles(state)

    def removePastWaypoints(self, state):
        self._omplPath.removePastWaypoints(state)
        self._latlons = self._getLatlonsFromOMPLPath(self._omplPath)

    # More complex methods
    def reachesGoalLatlon(self, goalLatlon):
        '''Checks if the given path reaches the given goalLatlon

        Args:
           goalLatlon (sailbot_msg.msg._latlon.latlon): Latlon that this path should be ending at

        Returns:
           bool True iff the distance between the goalLatlon and the last path waypoint is below a threshold
        '''
        lastWaypointLatlon = self._latlons[len(self._latlons) - 1]
        lastWaypoint = (lastWaypointLatlon.lat, lastWaypointLatlon.lon)
        goal = (goalLatlon.lat, goalLatlon.lon)
        return distance(lastWaypoint, goal).kilometers <= MAX_ALLOWABLE_DISTANCE_FINAL_WAYPOINT_TO_GOAL_KM

    def waypointReached(self, positionLatlon, previousWaypointLatlon, nextWaypointLatlon, isGlobal=False):
        '''Check if the given positionLatlon has reached the next waypoint.
        This is done by drawing a line from the waypoint at (self._nextWaypointIndex - 1) to (self._nextWaypointIndex)
        Then drawing a line perpendicular to the previous line that
        is dist in front of the waypoint at self._nextWaypointIndex.
        Then checks if the positionLatlon is past the perpendicular line or not

        Examples: B is boat. 0 is (self._nextWaypointIndex - 1). 1 is (self._nextWaypointIndex).

        Example 1 Waypoint not reached:
              |
              |
        0  B    1
              |
              |

        Example 2 Waypoint reached:
              |
              |
        0       1
              |
              |B

        Args:
           positionLatlon (sailbot_msg.msg._latlon.latlon): Latlon of the current boat position

        Returns:
           bool True iff the positionLatlon has reached the next waypoint
        '''
        # Convert from latlons to XY
        refLatlon = self._omplPath.getReferenceLatlon()

        positionX, positionY = utils.latlonToXY(positionLatlon, refLatlon)
        previousWaypointXY = utils.latlonToXY(previousWaypointLatlon, refLatlon)
        nextWaypointXY = utils.latlonToXY(nextWaypointLatlon, refLatlon)

        isStartNorth, isStartEast, normalSlope, b = getPerpLine(previousWaypointXY, nextWaypointXY, isGlobal)
        dist = GLOBAL_WAYPOINT_REACHED_DISTANCE if isGlobal else WAYPOINT_REACHED_DISTANCE

        # Handle edge cases where waypoints have the same x or y component
        if normalSlope == 0:
            if isStartNorth:
                return positionY <= b
            else:
                return positionY >= b
        elif not normalSlope:
            if isStartEast:
                return positionX <= nextWaypointXY[0] + dist
            else:
                return positionX >= nextWaypointXY[0] - dist

        def y(x):
            return normalSlope * x + b

        def x(y):
            return (y - b) / normalSlope

        # import numpy as np
        # plt.xlim(-20, 20)
        # plt.ylim(-20, 20)
        # plt.plot([0], [0], marker='o', markersize=10, color="black")
        # plt.plot([positionX], [positionY], marker='o', markersize=10, color="blue")
        # plt.plot([previousWaypointX], [previousWaypointY], marker='o', markersize=10, color="green")
        # plt.plot([nextWaypointX], [nextWaypointY], marker="o", markersize=10, color="red")
        # x_plot = np.linspace(-200, 200, 100)
        # plt.plot(x_plot, y(x_plot), '-r')
        # plt.show()

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

    def nextWaypointReached(self, positionLatlon):
        previousWaypointLatlon = self._latlons[self._nextWaypointIndex - 1]
        nextWaypointLatlon = self._latlons[self._nextWaypointIndex]

        return self.waypointReached(positionLatlon, previousWaypointLatlon, nextWaypointLatlon)

    # assumes there are at least 2 elements in self._latlons
    def lastWaypointReached(self, positionLatlon):
        previousWaypointLatlon = self._latlons[-2]
        nextWaypointLatlon = self._latlons[-1]

        return self.waypointReached(positionLatlon, previousWaypointLatlon, nextWaypointLatlon)

    def _cleanNumLookAheadWaypoints(self, numLookAheadWaypoints):
        '''Ensure nextLocalWaypointIndex + numLookAheadWaypoints is in bounds
        Default behavior when numLookAheadWaypoints is not given OR bad input: set to max

        Let path = [(0,0), (1,1), (2,2), (3,3), (4,4)], len(path) = 5

        If: self.getNextWaypointIndex() = 2
        maxNumLookAheadWaypoints = self.getLength() - self.getNextWaypointIndex() = 5 - 2 = 3

        If numLookAheadWaypoints = 2, then care about waypoints (2,2), (3,3)
        If numLookAheadWaypoints = 6, cuts it down to 3, then care about waypoints (2,2), (3,3), (4,4)
        '''
        maxNumLookAheadWaypoints = self.getLength() - self.getNextWaypointIndex()
        if numLookAheadWaypoints is None or numLookAheadWaypoints > maxNumLookAheadWaypoints:
            return maxNumLookAheadWaypoints
        return numLookAheadWaypoints

    def upwindOrDownwindOnPath(self, state, numLookAheadWaypoints=None, showWarnings=False):
        '''Checks if there exists any upwind or downwind sailing on the path starting
        from state.position to the next numLookAheadWaypoints

        Note: Requires that upwind/downwind sailing be detected consistently for a certain time interval to return True
              This prevents upwind/downwind to be detected from wind sensor noise or poor measurements while turning
              But will still catch real upwind/downwind sailing when called repeatedly

        Args:
           state (BoatState): state of the boat, which defines the boat position and the global wind
           numLookAheadWaypoints (int): number of waypoints to look ahead at from state.position
                                        (ignores upwind/downwind very far ahead)
           showWarnings (bool): display ros warnings if upwind/downwind is detected

        Returns:
           bool True iff this method detects upwind/downwind sailing on the path within the first numLookAheadWaypoints
           starting from state.position CONSISTENTLY for a certain time interval
        '''
        # Default behavior when numLookAheadWaypoints is not given OR bad input: set to max
        numLookAheadWaypoints = self._cleanNumLookAheadWaypoints(numLookAheadWaypoints)

        # Calculate global wind from measured wind and boat state
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)

        # Get relevant waypoints (boat position first, then the next
        # numLookAheadWaypoints startin from nextLocalWaypoint onwards)
        relevantWaypoints = []
        relevantWaypoints.append(utils.latlonToXY(state.position, self.getReferenceLatlon()))
        for waypointIndex in range(self.getNextWaypointIndex(), self.getNextWaypointIndex() + numLookAheadWaypoints):
            waypoint = self._omplPath.getSolutionPath().getState(waypointIndex)
            relevantWaypoints.append([waypoint.getX(), waypoint.getY()])

        # Check relevantWaypoints for upwind or downwind sailing
        upwindOrDownwind = False
        for waypointIndex in range(1, len(relevantWaypoints)):
            # Calculate required heading between waypoints
            waypoint = relevantWaypoints[waypointIndex]
            prevWaypoint = relevantWaypoints[waypointIndex - 1]
            requiredHeadingDegrees = math.degrees(math.atan2(waypoint[1] - prevWaypoint[1],
                                                             waypoint[0] - prevWaypoint[0]))

            if ph.isDownwind(math.radians(globalWindDirectionDegrees), math.radians(requiredHeadingDegrees)):
                if showWarnings:
                    rospy.loginfo("Downwind sailing on path detected. globalWindDirectionDegrees: {}. "
                                  "requiredHeadingDegrees: {}. waypointIndex: {}"
                                  .format(globalWindDirectionDegrees, requiredHeadingDegrees, waypointIndex))
                upwindOrDownwind = True
                break

            elif ph.isUpwind(math.radians(globalWindDirectionDegrees), math.radians(requiredHeadingDegrees)):
                if showWarnings:
                    rospy.loginfo("Upwind sailing on path detected. globalWindDirectionDegrees: {}. "
                                  "requiredHeadingDegrees: {}. waypointIndex: {}"
                                  .format(globalWindDirectionDegrees, requiredHeadingDegrees, waypointIndex))
                upwindOrDownwind = True
                break

        # Set counter to 0 on first use
        try:
            self.lastTimeNotUpwindOrDownwind
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
                rospy.logwarn("Upwind/downwind sailing detected for {} seconds consecutively, which is greater than"
                              "the {} second limit. This officially counts as upwind/downwind"
                              .format(consecutiveUpwindOrDownwindTimeSeconds, UPWIND_DOWNWIND_TIME_LIMIT_SECONDS))
            self.lastTimeNotUpwindOrDownwind = time.time()
            return True
        else:
            if showWarnings:
                rospy.loginfo("Upwind/downwind sailing detected for only {} seconds consecutively, which is less than"
                              "the {} second limit. This does not count as upwind/downwind sailing yet."
                              .format(consecutiveUpwindOrDownwindTimeSeconds, UPWIND_DOWNWIND_TIME_LIMIT_SECONDS))
            return False

    def obstacleOnPath(self, state, numLookAheadWaypoints=None, showWarnings=False):
        '''Checks if there any obstacles on the path starting from state.position to the next numLookAheadWaypoints

        Args:
           state (BoatState): state of the boat, which defines the boat position and obstacle positions
           numLookAheadWaypoints (int): number of waypoints to look ahead at from state.position
                                        (ignores obstacles very far ahead)
           showWarnings (bool): display ros warnings if obstacles are detected

        Returns:
           bool True iff there exists an obstacle on the path within the first numLookAheadWaypoints
           starting from state.position
        '''
        # Default behavior when numLookAheadWaypoints is not given OR bad input: set to max
        numLookAheadWaypoints = self._cleanNumLookAheadWaypoints(numLookAheadWaypoints)

        # Check if path will hit objects
        positionXY = utils.latlonToXY(state.position, self.getReferenceLatlon())

        self.updateObstacles(state)
        waypointIndexWithObstacle = indexOfObstacleOnPath(
            positionXY, self.getNextWaypointIndex(), numLookAheadWaypoints, self._omplPath)
        if waypointIndexWithObstacle != -1:
            if showWarnings:
                rospy.logwarn("Obstacle on path. waypointIndexWithObstacle: {}".format(waypointIndexWithObstacle))
            return True
        return False


def incrementCountInvalidSolutions():
    global count_invalid_solutions
    count_invalid_solutions += 1


def incrementTempInvalidSolutions():
    global temp_invalid_solutions
    temp_invalid_solutions += 1


def createPath(state, runtimeSeconds=1.0, numRuns=2, resetSpeedupDuringPlan=False, speedupBeforePlan=1.0,
               maxAllowableRuntimeSeconds=MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS):
    '''Create a Path from state.position to state.globalWaypoint. Runs OMPL pathfinding multiple times and returns
    the best path.

    Args:
       state (BoatState): Current state of the boat, which contains all necessary information to perform pathfinding
       runtimeSeconds (float): Number of seconds that the each pathfinding attempt should run
       numRuns (int): Number of pathfinding attempts that are run in normal case
       resetSpeedupDuringPlan (bool): Decides if pathfinding should set the speedup value to 1.0 during the pathfinding
       speedupBeforePlan (double): Only used if resetSpeedupDuringPlan is True. At the end of pathfinding,
                                   publishes this speedup value
       maxAllowableRuntimeSeconds (double): Maximum total time that this method should take to run. Will take longer
                                            than runtimeSeconds*numRuns only if pathfinding is unable to find a path.

    Returns:
       Path object representing the path from state.position to state.globalWaypoint
    '''
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

    def isValidSolution(solution, referenceLatlon, state):
        if not solution.haveExactSolutionPath():
            return False
        return True

    def plotPathfindingProblem(globalWindDirectionDegrees, dimensions, start, goal, obstacles, headingDegrees):
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
        axes.set_title('Setup of pathfinding problem')

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

    def setAverageDistanceBetweenWaypoints(solutionPath):
        # Set the average distance between waypoints
        localPathLengthKm = solutionPath.length()
        numberOfLocalWaypoints = int(localPathLengthKm / utils.AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM)
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

    ou.setLogLevel(ou.LOG_WARN)
    # Set speedup to 1.0 during planning
    if resetSpeedupDuringPlan:
        speedupDuringPlan = 1.0
        rospy.loginfo("Setting speedup to this value during planning = {}".format(speedupDuringPlan))
        publisher = rospy.Publisher('speedup', Float64, queue_size=4)
        publisher.publish(speedupDuringPlan)

    # Get setup parameters from state for ompl plan()
    # Convert all latlons to NE in km wrt referenceLatlon
    referenceLatlon = state.globalWaypoint
    start = utils.latlonToXY(state.position, referenceLatlon)
    goal = utils.latlonToXY(state.globalWaypoint, referenceLatlon)
    dimensions = getXYLimits(start, goal)
    obstacles = obs.getObstacles(state, referenceLatlon)
    globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
        state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)

    # Run the planner multiple times and find the best one
    rospy.loginfo("Running createLocalPathSS. runtimeSeconds: {}. numRuns: {}. Total time: {} seconds"
                  .format(runtimeSeconds, numRuns, runtimeSeconds * numRuns))

    # Create non-blocking plot showing the setup of the pathfinding problem.
    # Useful to understand if the pathfinding problem is invalid or impossible
    shouldPlot = rospy.get_param('plot_pathfinding_problem', False)
    if shouldPlot:
        plotPathfindingProblem(globalWindDirectionDegrees, dimensions, start, goal, obstacles,
                               state.headingDegrees)

    # Take screenshot
    shouldTakeScreenshot = rospy.get_param('screenshot', False)
    if shouldTakeScreenshot:
        utils.takeScreenshot()

    # Look for solutions
    validSolutions = []
    invalidSolutions = []
    plannerType = rospy.get_param('planner_type', 'RRTStar')
    for i in range(numRuns):
        # TODO: Incorporate globalWindSpeed into pathfinding?
        rospy.loginfo("Starting path-planning run number: {}".format(i))
        solution = plan(runtimeSeconds, plannerType, globalWindDirectionDegrees,
                        dimensions, start, goal, obstacles, state.headingDegrees)
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
        incrementTempInvalidSolutions()

        # If valid solution can't be found for large runtime, then stop searching
        if totalRuntimeSeconds >= maxAllowableRuntimeSeconds:
            rospy.logwarn("No valid solution can be found in under {} seconds. Using invalid solution."
                          .format(maxAllowableRuntimeSeconds))
            incrementCountInvalidSolutions()
            break

        rospy.logwarn("Attempting to rerun with longer runtime: {} seconds".format(runtimeSeconds))
        solution = plan(runtimeSeconds, plannerType, globalWindDirectionDegrees,
                        dimensions, start, goal, obstacles, state.headingDegrees)

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
