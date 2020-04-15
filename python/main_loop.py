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
    omplLocalPath = createOmplPath(state, plot=True, resetSpeedupDuringPlan=True, speedupBeforePlan=speedup, maxAllowableRuntimeSeconds=maxAllowableRuntimeSeconds)
    latlonLocalPath = LatlonPath(omplLocalPath)
    lastTimePathCreated = time.time()
    return omplLocalPath, latlonLocalPath, lastTimePathCreated

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
    omplLocalPath, latlonLocalPath, lastTimePathCreated = updateToNewLocalPath(state, MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS)
    sailbot.newGlobalPathReceived = False

    while not rospy.is_shutdown():
        rospy.loginfo("desiredHeadingMsg: {}".format(desiredHeadingMsg.headingDegrees))
        rospy.loginfo("Current path cost is: {}".format(omplLocalPath.getCost()))
        state = sailbot.getCurrentState()

        # Check if local path MUST be updated
        hasUpwindOrDownwindOnPath = upwindOrDownwindOnPath(state, latlonLocalPath.getNextWaypointIndex(), omplLocalPath, numLookAheadWaypoints=NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND, showWarnings=True)
        hasObstacleOnPath = obstacleOnPath(state, latlonLocalPath.getNextWaypointIndex(), omplLocalPath, numLookAheadWaypoints=NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES, showWarnings=True)
        isGlobalWaypointReached = globalWaypointReached(state.position, state.globalWaypoint)
        newGlobalPathReceived = sailbot.newGlobalPathReceived
        reachedEndOfLocalPath = latlonLocalPath.reachedEnd()
        pathNotReachGoal = not latlonLocalPath.reachesGoalLatlon(state.globalWaypoint)
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
            omplLocalPath, latlonLocalPath, lastTimePathCreated = updateToNewLocalPath(state, MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS / 2.0)

        else:
            # Check if new local path should be generated and compared to current local path
            costTooHigh = pathCostThresholdExceeded(omplLocalPath.getCost())
            isLocalWaypointReached = latlonLocalPath.nextLocalWaypointReached(state.position, omplLocalPath.getReferenceLatlon())
            isTimeLimitExceeded = timeLimitExceeded(lastTimePathCreated, speedup)

            if costTooHigh or isLocalWaypointReached or isTimeLimitExceeded or localPathUpdateRequested:
                rospy.loginfo("Generating new local path to compare to current local path. Reason: costTooHigh? {}. isLocalWaypointReached? {}. isTimeLimitExceeded? {}. localPathUpdateRequested? {}.".format(costTooHigh, isLocalWaypointReached, isTimeLimitExceeded, localPathUpdateRequested))

                # Reset request
                if localPathUpdateRequested:
                    localPathUpdateRequested = False

                # Remove previous waypoint
                if isLocalWaypointReached:
                    omplLocalPath.removePastWaypoints(state)
                    localPathLatlons = getLocalPathLatlons(omplLocalPath)
                    localWaypoint = getLocalWaypointLatLon(localPathLatlons, 1)

                # Update local path if new one is better than old
                _omplLocalPath, _latlonLocalPath, _lastTimePathCreated = updateToNewLocalPath(state, MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS)
                lastTimePathCreated = _lastTimePathCreated
                currentPathCost = omplLocalPath.getCost()
                newPathCost = _omplLocalPath.getCost()
                rospy.loginfo("currentPathCost = {}. newPathCost = {}".format(currentPathCost, newPathCost))
                if newPathCost < currentPathCost:
                    rospy.loginfo("Updating to new local path")
                    omplLocalPath, localPathLatlons, localWaypoint = _omplLocalPath, _localPathLatlons, _localWaypoint
                else:
                    rospy.loginfo("Keeping old local path")

        # Publish desiredHeading
        desiredHeadingMsg.headingDegrees = getDesiredHeading(state.position, localWaypoint)
        desiredHeadingPublisher.publish(desiredHeadingMsg)

        # Publish local path
        localPathPublisher.publish(msg.path(localPathLatlons))

        # Update wind direction and obstacles of omplLocalPath for proper cost calculation
        omplLocalPath.updateWindDirection(state)
        omplLocalPath.updateObstacles(state)

        # Publish nextLocalWaypoint and nextGlobalWaypoint and path cost
        nextLocalWaypointPublisher.publish(localWaypoint)
        nextGlobalWaypointPublisher.publish(state.globalWaypoint)
        pathCostPublisher.publish(omplLocalPath.getCost())
        pathCostBreakdownPublisher.publish(omplLocalPath.getPathCostBreakdownString())

        # If there are any plots, give some time for them to respond to requests, such as closing
        plt.pause(0.001)
        publishRate.sleep()
