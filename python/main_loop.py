#!/usr/bin/env python

import rospy
import sailbot_msg.msg as msg
import handleInvalidState as his
from std_msgs.msg import Float64, String, Bool
import Sailbot as sbot
import utilities as utils
import Path
import time
import matplotlib.pyplot as plt

# Constants
MAIN_LOOP_PERIOD_SECONDS = 0.5
SMALL_TURN_DEGREES = 10
LOGGED_LATLONS_PER_LINE = 3
CREATE_NEW_LOCAL_PATH_ATTEPT_THRESH = 40

# Global variable to receive path update request messages
localPathUpdateRequested = False

# Global variable to receive path update force messages
localPathUpdateForced = False

# Global variables related to lowWindConditions
createNewLocalPathAttempts = 0
lowWindConditions = False


def localPathUpdateRequestedCallback(data):
    global localPathUpdateRequested
    rospy.loginfo("localPathUpdateRequested message received.")
    localPathUpdateRequested = True


def localPathUpdateForcedCallback(data):
    global localPathUpdateForced
    rospy.loginfo("localPathUpdateForced message received.")
    localPathUpdateForced = True


def lowWindConditionsCallback(data):
    global lowWindConditions
    lowWindConditions = data.data


def createNewLocalPath(sailbot, maxAllowableRuntimeSeconds, desiredHeadingPublisher, prevLocalPath):
    '''Ensures sailbot is in a valid starting state, then creates a local path.
    If sailbot not in a valid state, use his.getValidStateMoveToSafetyIfNeeded to move to safety first.

    Args:
       sailbot (Sailbot): Sailbot object with which current state will be calculated
       maxAllowableRuntimeSeconds (float): max time that can be spent generating a path
       desiredHeadingPublisher (rospy.Publisher:msg.heading) publisher for desired heading of sailbot to move to safety

    Returns:
       Path object of local path AND time of path creation
    '''
    # Gets current state. If start position is invalid, moves out of the way of the other boat before continuing
    state = his.getValidStateMoveToSafetyIfNeeded(sailbot, desiredHeadingPublisher)

    # Composition of functions used every time when updating local path
    localPath = Path.createPath(state, resetSpeedupDuringPlan=True,
                                maxAllowableRuntimeSeconds=maxAllowableRuntimeSeconds)
    # TODO: Use counter and publishing period instead of time to work with speedup
    lastTimePathCreated = time.time()

    # Handle no path found with previous solution
    if localPath is None:
        rospy.logerr("Could not find a new localPath within {} seconds. Using previous localPath."
                     .format(maxAllowableRuntimeSeconds))
        return prevLocalPath, lastTimePathCreated

    # Only reset createNewLocalPathAttempts if new path is successfully created
    global createNewLocalPathAttempts
    createNewLocalPathAttempts = 0
    return localPath, lastTimePathCreated


if __name__ == '__main__':
    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = sbot.Sailbot(nodeName='local_pathfinding')

    # Get enable_los parameter
    enable_los = rospy.get_param('enable_los', default=False)

    # Subscribe to requestLocalPathUpdate
    rospy.Subscriber('requestLocalPathUpdate', Bool, localPathUpdateRequestedCallback)
    rospy.Subscriber('forceLocalPathUpdate', Bool, localPathUpdateForcedCallback)

    # Subscribe to lowWindConditions
    rospy.Subscriber('lowWindConditions', Bool, lowWindConditionsCallback)

    # Create ros publisher for the desired heading for the controller
    desiredHeadingPublisher = rospy.Publisher('desired_heading_degrees', msg.heading, queue_size=4)

    # Create other publishers
    localPathPublisher = rospy.Publisher('localPath', msg.path, queue_size=4)
    nextLocalWaypointPublisher = rospy.Publisher('nextLocalWaypoint', msg.latlon, queue_size=4)
    previousLocalWaypointPublisher = rospy.Publisher('previousLocalWaypoint', msg.latlon, queue_size=4)
    nextGlobalWaypointPublisher = rospy.Publisher('nextGlobalWaypoint', msg.latlon, queue_size=4)
    previousGlobalWaypointPublisher = rospy.Publisher('previousGlobalWaypoint', msg.latlon, queue_size=4)
    pathCostPublisher = rospy.Publisher('localPathCost', Float64, queue_size=4)
    pathCostBreakdownPublisher = rospy.Publisher('localPathCostBreakdown', String, queue_size=4)
    publishRate = rospy.Rate(1.0 / MAIN_LOOP_PERIOD_SECONDS)

    # Wait until first sensor data + globalPath is received
    sailbot.waitForFirstSensorDataAndGlobalPath()

    rospy.loginfo("Starting main loop")

    # Create first path and track time of updates
    state = sailbot.getCurrentState()

    # Publish global waypoints first so that they can be visualized
    nextGlobalWaypointPublisher.publish(state.globalWaypoint)
    previousGlobalWaypointPublisher.publish(sailbot.globalPath[sailbot.globalPathIndex - 1])

    # No prev path to fall back on, so give lots of time
    localPath, lastTimePathCreated = createNewLocalPath(sailbot, float('inf'), desiredHeadingPublisher, None)
    desiredHeadingDegrees = utils.getDesiredHeadingDegrees(state.position, localPath.getNextWaypoint())
    desiredHeadingDegreesNewCoordinates = utils.headingToBearingDegrees(desiredHeadingDegrees)
    sailbot.newGlobalPathReceived = False

    while not rospy.is_shutdown():
        rospy.loginfo("desiredHeadingDegrees: {} (0 = east, 90 = north) OR {} (0 = north, 90 = east)"
                      .format(desiredHeadingDegrees, desiredHeadingDegreesNewCoordinates))
        rospy.loginfo("Current path cost is: {}".format(localPath.getCost()))
        state = sailbot.getCurrentState()
        rospy.loginfo_throttle(60, "state = {}".format(state))  # Log state but throttle because too big to print often

        # Check if local path MUST be updated
        hasUpwindOrDownwindOnPath = localPath.upwindOrDownwindOnPath(
            state, numLookAheadWaypoints=utils.NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND, showWarnings=True)
        hasObstacleOnPath = localPath.obstacleOnPath(
            state, numLookAheadWaypoints=utils.NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES, showWarnings=True)

        # global waypoint reached when boat crosses either red or final blue line
        isLastWaypointReached = localPath.lastWaypointReached(state.position)
        nextGlobalWaypoint = sailbot.globalPath[sailbot.globalPathIndex]
        previousGlobalWaypoint = sailbot.globalPath[sailbot.globalPathIndex - 1]
        isGlobalWaypointReached = (localPath.waypointReached(state.position, previousGlobalWaypoint, nextGlobalWaypoint,
                                                             True) or isLastWaypointReached)

        if(lowWindConditions):
            # ensure we havent updated local path recently before permitting
            lowWindPermitsUpdate = (createNewLocalPathAttempts >= CREATE_NEW_LOCAL_PATH_ATTEPT_THRESH)
        else:
            lowWindPermitsUpdate = True
            createNewLocalPathAttempts = 0

        newGlobalPathReceived = sailbot.newGlobalPathReceived
        reachedEndOfLocalPath = localPath.reachedEnd()
        pathNotReachGoal = not localPath.reachesGoalLatlon(state.globalWaypoint)
        mustUpdateLocalPath = ((hasUpwindOrDownwindOnPath and lowWindPermitsUpdate)
                               or hasObstacleOnPath or isGlobalWaypointReached
                               or newGlobalPathReceived or reachedEndOfLocalPath or pathNotReachGoal
                               or localPathUpdateForced)
        if mustUpdateLocalPath:
            # Log reason for local path update
            rospy.logwarn("MUST update local Path. Reason: hasUpwindOrDownwindOnPath? {}. hasObstacleOnPath? {}. "
                          "isGlobalWaypointReached? {}. newGlobalPathReceived? {}. reachedEndOfLocalPath? {}. "
                          "pathNotReachGoal? {}. localPathUpdateForced? {}. lowWindPermitsUpdate? {}"
                          .format(hasUpwindOrDownwindOnPath, hasObstacleOnPath, isGlobalWaypointReached,
                                  newGlobalPathReceived, reachedEndOfLocalPath, pathNotReachGoal,
                                  localPathUpdateForced, lowWindPermitsUpdate))

            # Reset request
            if localPathUpdateForced:
                localPathUpdateForced = False

            # Reset sailbot newGlobalPathReceived boolean
            if newGlobalPathReceived:
                sailbot.newGlobalPathReceived = False

            # Reset sbot.goalWasInvalid boolean
            if sbot.goalWasInvalid:
                sbot.goalWasInvalid = False

            # If globalWaypoint reached, increment the index
            if isGlobalWaypointReached or newGlobalPathReceived:
                isFirstNextGlobalWaypoint = True
                while localPath.waypointReached(state.position, previousGlobalWaypoint, nextGlobalWaypoint, True) or \
                        (isFirstNextGlobalWaypoint and isLastWaypointReached):
                    if isFirstNextGlobalWaypoint:
                        isFirstNextGlobalWaypoint = False
                        rospy.loginfo("Global waypoint reached")
                    else:
                        rospy.logwarn('Already reached next global waypoint, incrementing global path index')

                    sailbot.globalPathIndex += 1
                    nextGlobalWaypoint = sailbot.globalPath[sailbot.globalPathIndex]
                    previousGlobalWaypoint = sailbot.globalPath[sailbot.globalPathIndex - 1]

            # Update local path
            localPath, lastTimePathCreated = createNewLocalPath(
                sailbot, Path.MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS / 2.0,
                desiredHeadingPublisher, localPath)

        else:
            # Check if new local path should be generated and compared to current local path
            costTooHigh = utils.pathCostThresholdExceeded(localPath)
            isLocalWaypointReached = localPath.nextWaypointReached(state.position)
            isTimeLimitExceeded = utils.timeLimitExceeded(lastTimePathCreated, rospy.get_param('speedup', default=1.0))

            generateNewLocalPathToCompare = ((costTooHigh and lowWindPermitsUpdate) or isLocalWaypointReached
                                             or isTimeLimitExceeded or localPathUpdateRequested)
            if generateNewLocalPathToCompare:
                rospy.loginfo("Generating new local path to compare to current local path. Reason: costTooHigh? {}. "
                              "isLocalWaypointReached? {}. isTimeLimitExceeded? {}. localPathUpdateRequested? {}."
                              .format(costTooHigh, isLocalWaypointReached, isTimeLimitExceeded,
                                      localPathUpdateRequested))

                # Reset request
                if localPathUpdateRequested:
                    localPathUpdateRequested = False

                # Remove previous waypoint
                if isLocalWaypointReached:
                    localPath.removePastWaypoints(state)

                # Create new local path to compare
                _localPath, _lastTimePathCreated = createNewLocalPath(
                    sailbot, Path.MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS / 2.0,
                    desiredHeadingPublisher, localPath)
                lastTimePathCreated = _lastTimePathCreated

                # Update local path if new one is better than old AND it reaches the goal AND the turn to it is small
                currentPathCost = localPath.getCost()
                newPathCost = _localPath.getCost()
                newPathReachesGoal = _localPath.reachesGoalLatlon(state.globalWaypoint)
                absAngleToNewPath = utils.absAngleDegreesBetweenLatlons(localPath.getNextWaypoint(),
                                                                        _localPath.getNextWaypoint(), state.position)
                rospy.loginfo("currentPathCost = {}. newPathCost = {}. newPathReachesGoal = {}. "
                              "absAngleToNewPath = {}.".format(currentPathCost, newPathCost, newPathReachesGoal,
                                                               absAngleToNewPath))
                if newPathCost < currentPathCost and newPathReachesGoal and absAngleToNewPath < SMALL_TURN_DEGREES:
                    rospy.loginfo("Updating to new local path")
                    localPath = _localPath
                else:
                    rospy.loginfo("Keeping old local path")

            elif ((hasUpwindOrDownwindOnPath or costTooHigh)
                  and (createNewLocalPathAttempts <= CREATE_NEW_LOCAL_PATH_ATTEPT_THRESH)):
                # Increase the counter if we did NOT update local path, but there WAS an "unimportant" reason to update.
                createNewLocalPathAttempts += 1

        # Publish desiredHeading
        if enable_los:
            desiredHeadingDegrees = utils.getLOSHeadingDegrees(state.position, localPath.getPreviousWaypoint(),
                                                               localPath.getNextWaypoint())
        else:
            desiredHeadingDegrees = utils.getDesiredHeadingDegrees(state.position, localPath.getNextWaypoint())

        # Internal local_pathfinding uses East=0, North=90
        # rostopic uses North=0, East=90
        desiredHeadingDegreesNewCoordinates = utils.headingToBearingDegrees(desiredHeadingDegrees)
        desiredHeadingPublisher.publish(msg.heading(desiredHeadingDegreesNewCoordinates))

        # Publish local path
        localPathPublisher.publish(msg.path(localPath.getLatlons()))

        # Format and log local path
        latLonsToLog = "Path latlons are:"

        for i in range(len(localPath.getLatlons())):
            if(i % LOGGED_LATLONS_PER_LINE == 0):
                latLonsToLog += "\n"

            latLon = localPath.getLatlons()[i]
            latLonsToLog += "(lat: {}, lon: {})".format(latLon.lat, latLon.lon)

            if(i < len(localPath.getLatlons()) - 1):
                latLonsToLog += ", "

        latLonsToLog += "."
        rospy.loginfo(latLonsToLog)

        # Update wind direction and obstacles of localPath for proper cost calculation
        localPath.updateWindDirection(state)
        localPath.updateObstacles(state)

        # Publish nextLocalWaypoint and nextGlobalWaypoint and path cost. Log nextGlobalWaypoint
        nextLocalWaypointPublisher.publish(localPath.getNextWaypoint())
        previousLocalWaypointPublisher.publish(localPath.getPreviousWaypoint())
        nextGlobalWaypointPublisher.publish(state.globalWaypoint)
        globalWaypointLogStr = "Next Global Waypoint is: (lat: {}, lon: {})"
        rospy.loginfo(globalWaypointLogStr.format(state.globalWaypoint.lat, state.globalWaypoint.lon))
        previousGlobalWaypointPublisher.publish(sailbot.globalPath[sailbot.globalPathIndex - 1])
        pathCostPublisher.publish(localPath.getCost())
        pathCostBreakdownPublisher.publish(localPath.getPathCostBreakdownString())

        # If there are any plots, give some time for them to respond to requests, such as closing
        plt.pause(0.001)
        publishRate.sleep()

    rospy.loginfo("No valid solutions found {} times" .format(Path.temp_invalid_solutions))
    rospy.loginfo("Used invalid solutions {} times" .format(Path.count_invalid_solutions))
