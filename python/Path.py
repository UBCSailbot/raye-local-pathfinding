import math
import time
import rospy
from updated_geometric_planner import indexOfObstacleOnPath
import planner_helpers as ph
import utilities as utils
from local_pathfinding.msg import latlon
from geopy.distance import distance


class OMPLPath:
    """ Class for storing an OMPL configuration, OMPL path, and the referenceLatlon

    Attributes:
        _ss (ompl.geometric._geometric.SimpleSetup): SimpleSetup object that contains information about the OMPLPath,
                                                     including its state space, space information, and cost function
                                                     used to generate the path.
        _solutionPath (ompl.geometric._geometric.PathGeometric): PathGeometric object that represents the path, in XY
                                                                 coordinates with respect to a referenceLatlon.
        _referenceLatlon (local_pathfinding.msg._latlon.latlon): latlon object that is the reference point with which
                                                                 the path's XY coordinates is set.
    """

    def __init__(self, ss, solutionPath, referenceLatlon):
        self._ss = ss
        self._solutionPath = solutionPath
        self._referenceLatlon = referenceLatlon

    # Simple getter methods
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

    def getPathCostBreakdownString(self):
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
        # Set wind direction for cost evaluation
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
        objective = self._ss.getOptimizationObjective()  # Assumes balanced objective

        for i in range(objective.getObjectiveCount()):
            if isinstance(objective.getObjective(i), ph.WindObjective):
                objective.getObjective(i).windDirectionDegrees = globalWindDirectionDegrees
                return

        rospy.logwarn("updateWindDirection() was unsuccessful. Wind direction was not updated")

    def updateObstacles(self, state):
        # Set the objects used to check which states in the space are valid
        obstacles = utils.getObstacles(state.AISData.ships, state.position, state.speedKmph, self._referenceLatlon)
        validity_checker = ph.ValidityChecker(self._ss.getSpaceInformation(), obstacles)
        self._ss.setStateValidityChecker(validity_checker)

    def removePastWaypoints(self, state):
        """Removes waypoints that the boat has already passed.

        Note: Best used when waypointReached() == True, not every loop, otherwise the
              localWaypointReached tangent line could get messed up.
        Additional reason: keepAfter() method implementation assumes all waypoints are equally spaced
        Algorithm:
        1. Find index of waypoint closest to state, call that closestWaypoint
        2. Check dist(waypoint before closestWaypoint, state) and dist(waypoint after closestWaypoint, state)
            * If closer to waypoint BEFORE closestWaypoint: remove all waypoints before closestWaypoint
              EXCLUDING closestWaypoint
            * If closer to waypoint AFTER closestWaypoint: remove all waypoints before closestWaypoint
              INCLUDING closestWaypoint
        This answers the question: should we remove the closestWaypoint?
        Eg. Are we past the closestWaypoint or behind it?

        Algorithm works when all waypoints have about the same distance apart.
        Doesn't always work when waypoints are not all equally close

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
        If (0 waypoints are removed) OR (1 waypoint is removed AND dist(B, 1) < dist(0,1)),
        then dont add position B as starting waypoint.
        Else add position B as a starting waypoint.
        """

        # Get current position in xy coordinates
        x, y = utils.latlonToXY(state.position, self._referenceLatlon)
        positionXY = self._ss.getSpaceInformation().allocState()
        positionXY.setXY(x, y)

        # Keep waypoints only after your positionXY
        lengthBefore = self._solutionPath.getStateCount()
        self._solutionPath.keepAfter(positionXY)
        lengthAfter = self._solutionPath.getStateCount()

        def dist(state1, state2):
            """Calculates the euclidean distance between two states.

            Keyword arguments:
                state1 (ompl.base._base.SE2StateInternal): SE2State object of first state
                state2 (ompl.base._base.SE2StateInternal): SE2State object of second state

            Note: Do not replace this method with self._ss.getStateSpace().distance(state1, state2)
                  as this method also takes into account angle differences between these states,
                  which is not relevant here.
            """
            x1 = state1.getX()
            y1 = state1.getY()
            x2 = state2.getX()
            y2 = state2.getY()
            return ((x1-x2)**2 + (y1-y2)**2)**0.5

        # Only add in boat position as waypoint if edge case is avoided (described above)
        dist_boat_to_1 = dist(positionXY, self._solutionPath.getState(1))
        dist_0_to_1 = dist(self._solutionPath.getState(0), self._solutionPath.getState(1))
        boatCouldGoWrongDirection = dist_boat_to_1 < dist_0_to_1
        rospy.loginfo("dist_boat_to_1 = {}. dist_0_to_1 = {}. boatCouldGoWrongDirection = {}."
                      .format(dist_boat_to_1, dist_0_to_1, boatCouldGoWrongDirection))

        edgeCase = ((lengthBefore - lengthAfter == 0) or
                    ((lengthBefore - lengthAfter == 1) and boatCouldGoWrongDirection))
        if edgeCase:
            rospy.loginfo("Thus edgeCase = {}, so positionXY not prepended to path.".format(edgeCase))
        else:
            self._solutionPath.prepend(positionXY)
            rospy.loginfo("Thus edgeCase = {}, so positionXY was prepended to path.".format(edgeCase))


class Path:
    """ Class for storing and interacting with a path in both latlon and NE (xy) coordinates

    Attributes:
        _omplPath (OMPLPath): SimpleSetup object that contains information about the OMPLPath,
                                                     including its state space, space information, and cost function
                                                     used to generate the path.
        _latlons (): PathGeometric object that represents the path, in XY
                                                                 coordinates with respect to a referenceLatlon.
        _nextWaypointIndex (int): latlon object that is the reference point with which
                                                                 the path's XY coordinates is set.
    """
    def __init__(self, omplPath):
        self._omplPath = omplPath
        self._latlons = self._getLatlonsFromOMPLPath(self._omplPath)

        # Next waypoint index is always 1, as the boat should always be aiming for the next upcoming waypoint
        self._nextWaypointIndex = 1
        print(type(self._omplPath))
        print(type(self._latlons))
        print(type(self._nextWaypointIndex))

    def _getLatlonsFromOMPLPath(self, omplPath):
        # Convert solution path (in km WRT reference) into list of latlons
        path = []
        solutionPathObject = omplPath.getSolutionPath()
        referenceLatlon = omplPath.getReferenceLatlon()
        for state in solutionPathObject.getStates():
            xy = (state.getX(), state.getY())
            path.append(utils.XYToLatlon(xy, referenceLatlon))
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
        return distance(lastWaypoint, goal).kilometers <= utils.MAX_ALLOWABLE_DISTANCE_FINAL_WAYPOINT_TO_GOAL_KM

    def nextWaypointReached(self, positionLatlon):
        # Convert from latlons to XY
        refLatlon = self._omplPath.getReferenceLatlon()
        previousWaypointLatlon = self._latlons[self._nextWaypointIndex - 1]
        nextWaypointLatlon = self._latlons[self._nextWaypointIndex]

        positionX, positionY = utils.latlonToXY(positionLatlon, refLatlon)
        previousWaypointX, previousWaypointY = utils.latlonToXY(previousWaypointLatlon, refLatlon)
        nextWaypointX, nextWaypointY = utils.latlonToXY(nextWaypointLatlon, refLatlon)

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
        normalSlope = -1 / tangentSlope

        if nextWaypointX > 0:
            b = nextWaypointY + normalSlope * -math.fabs(nextWaypointX)
        else:
            b = nextWaypointY + normalSlope * math.fabs(nextWaypointX)

        def y(x): return normalSlope * x + b
        def x(y): return (y - b) / normalSlope

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
        if consecutiveUpwindOrDownwindTimeSeconds >= utils.UPWIND_DOWNWIND_TIME_LIMIT_SECONDS:
            if showWarnings:
                rospy.logwarn("Upwind/downwind sailing detected for {} seconds consecutively, which is greater than"
                              "the {} second limit. This officially counts as upwind/downwind"
                              .format(consecutiveUpwindOrDownwindTimeSeconds, utils.UPWIND_DOWNWIND_TIME_LIMIT_SECONDS))
            self.lastTimeNotUpwindOrDownwind = time.time()
            return True
        else:
            if showWarnings:
                rospy.loginfo("Upwind/downwind sailing detected for only {} seconds consecutively, which is less than"
                              "the {} second limit. This does not count as upwind/downwind sailing yet."
                              .format(consecutiveUpwindOrDownwindTimeSeconds, utils.UPWIND_DOWNWIND_TIME_LIMIT_SECONDS))
            return False

    def obstacleOnPath(self, state, numLookAheadWaypoints=None, showWarnings=False):
        # Check if path will hit objects
        positionXY = utils.latlonToXY(state.position, self.getReferenceLatlon())

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
        waypointIndexWithObstacle = indexOfObstacleOnPath(
            positionXY, self.getNextWaypointIndex(), numLookAheadWaypoints, self._omplPath)
        if waypointIndexWithObstacle != -1:
            if showWarnings:
                rospy.logwarn("Obstacle on path. waypointIndexWithObstacle: {}".format(waypointIndexWithObstacle))
            return True
        return False
