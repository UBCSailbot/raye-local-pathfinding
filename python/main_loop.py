#!/usr/bin/env python

import sys
import rospy
import local_pathfinding.msg as msg
from std_msgs.msg import Float64, String, Bool
from Sailbot import *
from utilities import *
import time

# Constants
MAIN_LOOP_PERIOD_SECONDS = 0.1

# Global variable to receive path update requests
localPathUpdateRequested = False
def updatePathCallback(data):
    global localPathUpdateRequested
    rospy.loginfo("localPathUpdateRequested message received.")
    localPathUpdateRequested = True

# Global variable for speedup
speedup = 1.0
def speedupCallback(data):
    global speedup
    speedup = data.data

if __name__ == '__main__':
    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = Sailbot(nodeName='local_pathfinding')

    # Subscribe to requestLocalPathUpdate
    rospy.Subscriber('requestLocalPathUpdate', Bool, updatePathCallback)
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
    localPathSS, solutionPathObject, referenceLatlon, currentCost = createLocalPathSS(state, plot=True, resetSpeedupDuringPlan=True, speedupBeforePlan=speedup, maxAllowableRuntimeSeconds=MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS)
    localPathLatlons = getLocalPathLatlons(solutionPathObject, referenceLatlon)
    localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
    localWaypoint = getLocalWaypointLatLon(localPathLatlons, localPathIndex)
    lastTimePathCreated = time.time()
    sailbot.newGlobalPathReceived = False

    while not rospy.is_shutdown():
        rospy.loginfo("sailbot.globalPathIndex: {}".format(sailbot.globalPathIndex))
        rospy.loginfo("localPathIndex: {}".format(localPathIndex))
        rospy.loginfo("desiredHeadingMsg: {}".format(desiredHeadingMsg.headingDegrees))
        rospy.loginfo("Current path cost is: {}".format(currentCost))
        state = sailbot.getCurrentState()

        # Check if local path MUST be updated
        hasUpwindOrDownwindOnPath = upwindOrDownwindOnPath(state, localPathIndex, solutionPathObject, referenceLatlon, numLookAheadWaypoints=NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND, showWarnings=True)
        hasObstacleOnPath = obstacleOnPath(state, localPathIndex, localPathSS, solutionPathObject, referenceLatlon, numLookAheadWaypoints=NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES, showWarnings=True)
        isGlobalWaypointReached = globalWaypointReached(state.position, state.globalWaypoint)
        newGlobalPathReceived = sailbot.newGlobalPathReceived
        localPathIndexOutOfBounds = localPathIndex >= len(localPathLatlons)
        pathNotReachGoal = pathDoesNotReachGoal(localPathLatlons, state.globalWaypoint)
        if hasUpwindOrDownwindOnPath or hasObstacleOnPath or isGlobalWaypointReached or newGlobalPathReceived or localPathIndexOutOfBounds or pathNotReachGoal:
            # Log reason for local path update
            rospy.logwarn("MUST update local Path. Reason: hasUpwindOrDownwindOnPath? {}. hasObstacleOnPath? {}. isGlobalWaypointReached? {}. newGlobalPathReceived? {}. localPathIndexOutOfBounds? {}. pathNotReachGoal? {}".format(hasUpwindOrDownwindOnPath, hasObstacleOnPath, isGlobalWaypointReached, newGlobalPathReceived, localPathIndexOutOfBounds, pathNotReachGoal))

            # Reset saiblot newGlobalPathReceived boolean
            if localPathUpdateRequested:
                localPathUpdateRequested = False

            # Reset saiblot newGlobalPathReceived boolean
            if newGlobalPathReceived:
                sailbot.newGlobalPathReceived = False

            # If globalWaypoint reached, increment the index
            if isGlobalWaypointReached:
                rospy.loginfo("Global waypoint reached")
                sailbot.globalPathIndex += 1

            # Update local path
            state = sailbot.getCurrentState()
            localPathSS, solutionPathObject, referenceLatlon, currentCost = createLocalPathSS(state, plot=True, resetSpeedupDuringPlan=True, speedupBeforePlan=speedup, maxAllowableRuntimeSeconds=MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS / 2)
            localPathLatlons = getLocalPathLatlons(solutionPathObject, referenceLatlon)
            localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
            localWaypoint = getLocalWaypointLatLon(localPathLatlons, localPathIndex)
            lastTimePathCreated = time.time()
        else:
            # Check if new local path should be generated and compared to current local path
            costTooHigh = pathCostThresholdExceeded(currentCost)
            isLocalWaypointReached = localWaypointReached(state.position, localPathLatlons, localPathIndex, referenceLatlon)
            isTimeLimitExceeded = timeLimitExceeded(lastTimePathCreated, speedup)

            if costTooHigh or isLocalWaypointReached or isTimeLimitExceeded:
                rospy.logwarn("Generating new local path to compare to current local path. Reason: costTooHigh? {}. isLocalWaypointReached? {}. isTimeLimitExceeded? {}.".format(costTooHigh, isLocalWaypointReached, isTimeLimitExceeded))

                # Remove previous waypoint
                if isLocalWaypointReached:
                    removePastWaypointsInSolutionPath(localPathSS, solutionPathObject, referenceLatlon, state)
                    localPathIndex = 1
                    localPathLatlons = getLocalPathLatlons(solutionPathObject, referenceLatlon)
                    localWaypoint = getLocalWaypointLatLon(localPathLatlons, localPathIndex)

                # Update local path if new one is better than old
                _localPathSS, _solutionPathObject, _referenceLatlon, newCost = createLocalPathSS(state, plot=True, resetSpeedupDuringPlan=True, speedupBeforePlan=speedup, maxAllowableRuntimeSeconds=MAX_ALLOWABLE_PATHFINDING_TOTAL_RUNTIME_SECONDS)
                lastTimePathCreated = time.time()
                rospy.loginfo("currentCost = {}. newCost = {}".format(currentCost, newCost))
                if newCost < currentCost:
                    rospy.loginfo("Updating to new local path")
                    localPathSS, solutionPathObject, referenceLatlon = _localPathSS, _solutionPathObject, _referenceLatlon
                    localPathLatlons = getLocalPathLatlons(solutionPathObject, referenceLatlon)
                    localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
                    localWaypoint = getLocalWaypointLatLon(localPathLatlons, localPathIndex)
                    currentCost = newCost
                else:
                    rospy.loginfo("Keeping old local path")

        # Publish desiredHeading
        desiredHeadingMsg.headingDegrees = getDesiredHeading(state.position, localWaypoint)
        desiredHeadingPublisher.publish(desiredHeadingMsg)

        # Publish local path
        localPathPublisher.publish(msg.path(localPathLatlons))

        # Update wind direction and obstacles of localPathSS
        updateWindDirectionInSS(localPathSS, state)
        updateObjectsInSS(localPathSS, referenceLatlon, state)
        currentCost = solutionPathObject.cost(localPathSS.getOptimizationObjective()).value()

        # Publish nextLocalWaypoint and nextGlobalWaypoint and path cost
        nextLocalWaypointPublisher.publish(localWaypoint)
        nextGlobalWaypointPublisher.publish(state.globalWaypoint)
        pathCostPublisher.publish(solutionPathObject.cost(localPathSS.getOptimizationObjective()).value())
        pathCostBreakdownPublisher.publish(getPathCostBreakdownString(localPathSS.getOptimizationObjective(), solutionPathObject))

        # If there are any plots, give some time for them to respond to requests, such as closing
        plt.pause(0.001)
        publishRate.sleep()
