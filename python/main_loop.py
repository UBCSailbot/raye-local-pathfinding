#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Bool
import local_pathfinding.msg as msg
from std_msgs.msg import Float64, String, Bool
from Sailbot import *
from utilities import *
import time

# Constants
MAIN_LOOP_PERIOD_SECONDS = 0.5

# Global variable to receive path update request messages
localPathUpdateRequested = False
def localPathUpdateRequestedCallback(data):
    global localPathUpdateRequested
    rospy.loginfo("localPathUpdateRequested message received.")
    localPathUpdateRequested = True

# Global variable to receive path update force messages
localPathUpdateForced = False
def localPathUpdateForcedCallback(data):
    global localPathUpdateForced
    rospy.loginfo("localPathUpdateForced message received.")
    localPathUpdateForced = True

# Global variable for speedup
speedup = 1.0
def speedupCallback(data):
    global speedup
    speedup = data.data

def updateToNewLocalPath(state, maxAllowableRuntimeSeconds):
    # Composition of functions used every time when updating local path
    global speedup
    localPath = createPath(state, resetSpeedupDuringPlan=True, speedupBeforePlan=speedup, maxAllowableRuntimeSeconds=maxAllowableRuntimeSeconds)
    lastTimePathCreated = time.time()
    return localPath, lastTimePathCreated

if __name__ == '__main__':
    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = Sailbot(nodeName='local_pathfinding')

    # Subscribe to requestLocalPathUpdate
    rospy.Subscriber('requestLocalPathUpdate', Bool, localPathUpdateRequestedCallback)
    rospy.Subscriber('forceLocalPathUpdate', Bool, localPathUpdateForcedCallback)
    rospy.Subscriber('speedup', Float64, speedupCallback)

    # Create ros publisher for the desired heading for the controller
    desiredHeadingPublisher = rospy.Publisher('desiredHeading', msg.heading, queue_size=4)
    desiredHeadingMsg = msg.heading()

    # Create other publishers
    localPathPublisher = rospy.Publisher('localPath', msg.path, queue_size=4)
    nextLocalWaypointPublisher = rospy.Publisher('nextLocalWaypoint', msg.latlon, queue_size=4)
    nextGlobalWaypointPublisher = rospy.Publisher('nextGlobalWaypoint', msg.latlon, queue_size=4)
    pathCostPublisher = rospy.Publisher('localPathCost', Float64, queue_size=4)
    pathCostBreakdownPublisher = rospy.Publisher('localPathCostBreakdown', String, queue_size=4)
    publishRate = rospy.Rate(1.0 / MAIN_LOOP_PERIOD_SECONDS)

    # Wait until first global path is received
    waitForGlobalPath(sailbot)
    rospy.loginfo("Starting main loop")

    # Create first path and track time of updates
    state = sailbot.getCurrentState()
    localPath, lastTimePathCreated = updateToNewLocalPath(state, MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS)
    sailbot.newGlobalPathReceived = False

    while not rospy.is_shutdown():
        rospy.loginfo("desiredHeadingMsg: {}".format(desiredHeadingMsg.headingDegrees))
        rospy.loginfo("Current path cost is: {}".format(localPath.getCost()))
        state = sailbot.getCurrentState()

        # Check if local path MUST be updated
        hasUpwindOrDownwindOnPath = localPath.upwindOrDownwindOnPath(state, numLookAheadWaypoints=NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND, showWarnings=True)
        hasObstacleOnPath = localPath.obstacleOnPath(state, numLookAheadWaypoints=NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES, showWarnings=True)
        isGlobalWaypointReached = globalWaypointReached(state.position, state.globalWaypoint)
        newGlobalPathReceived = sailbot.newGlobalPathReceived
        reachedEndOfLocalPath = localPath.reachedEnd()
        pathNotReachGoal = not localPath.reachesGoalLatlon(state.globalWaypoint)
        if hasUpwindOrDownwindOnPath or hasObstacleOnPath or isGlobalWaypointReached or newGlobalPathReceived or reachedEndOfLocalPath or pathNotReachGoal or localPathUpdateForced:
            # Log reason for local path update
            rospy.logwarn("MUST update local Path. Reason: hasUpwindOrDownwindOnPath? {}. hasObstacleOnPath? {}. isGlobalWaypointReached? {}. newGlobalPathReceived? {}. reachedEndOfLocalPath? {}. pathNotReachGoal? {}. localPathUpdateForced? {}.".format(hasUpwindOrDownwindOnPath, hasObstacleOnPath, isGlobalWaypointReached, newGlobalPathReceived, reachedEndOfLocalPath, pathNotReachGoal, localPathUpdateForced))

            # Reset request
            if localPathUpdateForced:
                localPathUpdateForced = False

            # Reset sailbot newGlobalPathReceived boolean
            if newGlobalPathReceived:
                sailbot.newGlobalPathReceived = False

            # If globalWaypoint reached, increment the index
            if isGlobalWaypointReached:
                rospy.loginfo("Global waypoint reached")
                sailbot.globalPathIndex += 1

            # Update local path
            state = sailbot.getCurrentState()
            localPath, lastTimePathCreated = updateToNewLocalPath(state, MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS / 2.0)

        else:
            # Check if new local path should be generated and compared to current local path
            costTooHigh = pathCostThresholdExceeded(localPath.getCost())
            isLocalWaypointReached = localPath.nextWaypointReached(state.position)
            isTimeLimitExceeded = timeLimitExceeded(lastTimePathCreated, speedup)

            if costTooHigh or isLocalWaypointReached or isTimeLimitExceeded or localPathUpdateRequested:
                rospy.loginfo("Generating new local path to compare to current local path. Reason: costTooHigh? {}. isLocalWaypointReached? {}. isTimeLimitExceeded? {}. localPathUpdateRequested? {}.".format(costTooHigh, isLocalWaypointReached, isTimeLimitExceeded, localPathUpdateRequested))

                # Reset request
                if localPathUpdateRequested:
                    localPathUpdateRequested = False

                # Remove previous waypoint
                if isLocalWaypointReached:
                    localPath.removePastWaypoints(state)

                # Update local path if new one is better than old
                _localPath, _lastTimePathCreated = updateToNewLocalPath(state, MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS)
                lastTimePathCreated = _lastTimePathCreated
                currentPathCost = localPath.getCost()
                newPathCost = _localPath.getCost()
                rospy.loginfo("currentPathCost = {}. newPathCost = {}".format(currentPathCost, newPathCost))
                if newPathCost < currentPathCost:
                    rospy.loginfo("Updating to new local path")
                    localPath = _localPath
                else:
                    rospy.loginfo("Keeping old local path")

        # Publish desiredHeading
        desiredHeadingMsg.headingDegrees = getDesiredHeadingDegrees(state.position, localPath.getNextWaypoint())
        desiredHeadingPublisher.publish(desiredHeadingMsg)

        # Publish local path
        localPathPublisher.publish(msg.path(localPath.getLatlons()))

        # Update wind direction and obstacles of localPath for proper cost calculation
        localPath.updateWindDirection(state)
        localPath.updateObstacles(state)

        # Publish nextLocalWaypoint and nextGlobalWaypoint and path cost
        nextLocalWaypointPublisher.publish(localPath.getNextWaypoint())
        nextGlobalWaypointPublisher.publish(state.globalWaypoint)
        pathCostPublisher.publish(localPath.getCost())
        pathCostBreakdownPublisher.publish(localPath.getPathCostBreakdownString())

        # If there are any plots, give some time for them to respond to requests, such as closing
        plt.pause(0.001)
        publishRate.sleep()
