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

# Global variable to receive path update request messages
localPathUpdateRequested = False

# Global variable to receive path update force messages
localPathUpdateForced = False

# Global variable for speedup
speedup = 1.0


def localPathUpdateRequestedCallback(data):
    global localPathUpdateRequested
    rospy.loginfo("localPathUpdateRequested message received.")
    localPathUpdateRequested = True


def localPathUpdateForcedCallback(data):
    global localPathUpdateForced
    rospy.loginfo("localPathUpdateForced message received.")
    localPathUpdateForced = True


def speedupCallback(data):
    global speedup
    speedup = data.data
    rospy.loginfo("speedup message of {} received.".format(speedup))


def createNewLocalPath(sailbot, maxAllowableRuntimeSeconds, desiredHeadingPublisher):
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
    global speedup
    localPath = Path.createPath(state, resetSpeedupDuringPlan=True, speedupBeforePlan=speedup,
                                maxAllowableRuntimeSeconds=maxAllowableRuntimeSeconds)
    lastTimePathCreated = time.time()
    return localPath, lastTimePathCreated


if __name__ == '__main__':
    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = sbot.Sailbot(nodeName='local_pathfinding')

    # Subscribe to requestLocalPathUpdate
    rospy.Subscriber('requestLocalPathUpdate', Bool, localPathUpdateRequestedCallback)
    rospy.Subscriber('forceLocalPathUpdate', Bool, localPathUpdateForcedCallback)
    rospy.Subscriber('speedup', Float64, speedupCallback)

    # Create ros publisher for the desired heading for the controller
    desiredHeadingPublisher = rospy.Publisher('desiredHeading', msg.heading, queue_size=4)

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

    # Set the initial speedup value. Best done here in main_loop to ensure the timing.
    # Should be published before loop begins.
    utils.setInitialSpeedup()

    rospy.loginfo("Starting main loop")

    # Create first path and track time of updates
    state = sailbot.getCurrentState()
    localPath, lastTimePathCreated = createNewLocalPath(sailbot, Path.MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS,
                                                        desiredHeadingPublisher)
    desiredHeadingDegrees = utils.getDesiredHeadingDegrees(state.position, localPath.getNextWaypoint())
    sailbot.newGlobalPathReceived = False

    while not rospy.is_shutdown():
        rospy.loginfo("desiredHeadingDegrees: {}".format(desiredHeadingDegrees))
        rospy.loginfo("Current path cost is: {}".format(localPath.getCost()))
        state = sailbot.getCurrentState()

        # Check if local path MUST be updated
        hasUpwindOrDownwindOnPath = localPath.upwindOrDownwindOnPath(
            state, numLookAheadWaypoints=utils.NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND, showWarnings=True)
        hasObstacleOnPath = localPath.obstacleOnPath(
            state, numLookAheadWaypoints=utils.NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES, showWarnings=True)
        isLastWaypointReached = localPath.lastWaypointReached(state.position)
        nextGlobalWaypoint = sailbot.globalPath[sailbot.globalPathIndex]
        previousGlobalWaypoint = sailbot.globalPath[sailbot.globalPathIndex - 1]
        isGlobalWaypointReached = (localPath.waypointReached(state.position, previousGlobalWaypoint, nextGlobalWaypoint)
                                   or isLastWaypointReached)  # true when boat crosses either red or final blue line
        newGlobalPathReceived = sailbot.newGlobalPathReceived
        reachedEndOfLocalPath = localPath.reachedEnd()
        pathNotReachGoal = not localPath.reachesGoalLatlon(state.globalWaypoint)
        goalInvalid = (not his.checkGoalValidity(state))
        mustUpdateLocalPath = (hasUpwindOrDownwindOnPath or hasObstacleOnPath or isGlobalWaypointReached
                               or newGlobalPathReceived or reachedEndOfLocalPath or pathNotReachGoal
                               or goalInvalid or localPathUpdateForced)
        if mustUpdateLocalPath:
            # Log reason for local path update
            rospy.logwarn("MUST update local Path. Reason: hasUpwindOrDownwindOnPath? {}. hasObstacleOnPath? {}. "
                          "isGlobalWaypointReached? {}. newGlobalPathReceived? {}. reachedEndOfLocalPath? {}. "
                          "pathNotReachGoal? {}. goalInvalid? {}. localPathUpdateForced? {}."
                          .format(hasUpwindOrDownwindOnPath, hasObstacleOnPath, isGlobalWaypointReached,
                                  newGlobalPathReceived, reachedEndOfLocalPath, pathNotReachGoal,
                                  goalInvalid, localPathUpdateForced))

            # Reset request
            if localPathUpdateForced:
                localPathUpdateForced = False

            # Reset sailbot newGlobalPathReceived boolean
            if newGlobalPathReceived:
                sailbot.newGlobalPathReceived = False

            # If globalWaypoint reached, increment the index
            if isGlobalWaypointReached or goalInvalid:
                if isGlobalWaypointReached:
                    rospy.loginfo("Global waypoint reached")
                if goalInvalid:
                    rospy.logwarn("Goal state invalid, skipping to the next global waypoint")
                sailbot.globalPathIndex += 1

            # Update local path
            localPath, lastTimePathCreated = createNewLocalPath(
                sailbot, Path.MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS / 2.0,
                desiredHeadingPublisher)

        else:
            # Check if new local path should be generated and compared to current local path
            costTooHigh = utils.pathCostThresholdExceeded(localPath)
            isLocalWaypointReached = localPath.nextWaypointReached(state.position)
            isTimeLimitExceeded = utils.timeLimitExceeded(lastTimePathCreated, speedup)

            generateNewLocalPathToCompare = (costTooHigh or isLocalWaypointReached or isTimeLimitExceeded
                                             or localPathUpdateRequested)
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
                    sailbot, Path.MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS,
                    desiredHeadingPublisher)
                lastTimePathCreated = _lastTimePathCreated

                # Update local path if new one is better than old AND it reaches the goal
                currentPathCost = localPath.getCost()
                newPathCost = _localPath.getCost()
                newPathReachesGoal = _localPath.reachesGoalLatlon(state.globalWaypoint)
                rospy.loginfo("currentPathCost = {}. newPathCost = {}. newPathReachesGoal = {}."
                              .format(currentPathCost, newPathCost, newPathReachesGoal))
                if newPathCost < currentPathCost and newPathReachesGoal:
                    rospy.loginfo("Updating to new local path")
                    localPath = _localPath
                else:
                    rospy.loginfo("Keeping old local path")

        # Publish desiredHeading
        desiredHeadingDegrees = utils.getDesiredHeadingDegrees(state.position, localPath.getNextWaypoint())
        desiredHeadingPublisher.publish(msg.heading(desiredHeadingDegrees))

        # Publish local path
        localPathPublisher.publish(msg.path(localPath.getLatlons()))

        # Update wind direction and obstacles of localPath for proper cost calculation
        localPath.updateWindDirection(state)
        localPath.updateObstacles(state)

        # Publish nextLocalWaypoint and nextGlobalWaypoint and path cost
        nextLocalWaypointPublisher.publish(localPath.getNextWaypoint())
        previousLocalWaypointPublisher.publish(localPath.getPreviousWaypoint())
        nextGlobalWaypointPublisher.publish(state.globalWaypoint)
        previousGlobalWaypointPublisher.publish(sailbot.globalPath[sailbot.globalPathIndex - 1])
        pathCostPublisher.publish(localPath.getCost())
        pathCostBreakdownPublisher.publish(localPath.getPathCostBreakdownString())

        # If there are any plots, give some time for them to respond to requests, such as closing
        plt.pause(0.001)
        publishRate.sleep()

    rospy.loginfo("No valid solutions found {} times" .format(Path.temp_invalid_solutions))
    rospy.loginfo("Used invalid solutions {} times" .format(Path.count_invalid_solutions))
