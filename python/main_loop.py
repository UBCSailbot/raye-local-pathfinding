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
    while not sailbot.newGlobalPathReceived:
        # Exit if shutdown
        if rospy.is_shutdown():
            rospy.loginfo("rospy.is_shutdown() is True. Exiting")
            sys.exit()
        else:
            rospy.loginfo("Waiting for sailbot to receive first newGlobalPath")
            time.sleep(1)
    rospy.loginfo("newGlobalPath received. Starting main loop")

    # Create first path and track time of updates
    state = sailbot.getCurrentState()
    localPathSS, solutionPathObject, referenceLatlon = createLocalPathSS(state, plot=True, resetSpeedupDuringPlan=True, speedupBeforePlan=speedup)
    localPathLatlons = getLocalPathLatlons(solutionPathObject, referenceLatlon)
    localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
    localWaypoint = getLocalWaypointLatLon(localPathLatlons, localPathIndex)
    lastTimePathCreated = time.time()
    sailbot.newGlobalPathReceived = False

    while not rospy.is_shutdown():
        rospy.loginfo("sailbot.globalPathIndex: {}".format(sailbot.globalPathIndex))
        rospy.loginfo("localPathIndex: {}".format(localPathIndex))
        rospy.loginfo("desiredHeadingMsg: {}".format(desiredHeadingMsg.headingDegrees))
        state = sailbot.getCurrentState()

        currentCost = solutionPathObject.cost(localPathSS.getOptimizationObjective()).value()
        rospy.logerr("Current path cost is: {}".format(currentCost))

        # Generate new local path if needed
        hasUpwindOrDownwindOnPath = upwindOrDownwindOnPath(state, localPathIndex, solutionPathObject, referenceLatlon, numLookAheadWaypoints=NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND, showWarnings=True)
        hasObstacleOnPath = obstacleOnPath(state, localPathIndex, localPathSS, solutionPathObject, referenceLatlon, numLookAheadWaypoints=NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES, showWarnings=True)
        isTimeLimitExceeded = timeLimitExceeded(lastTimePathCreated, speedup)
        isGlobalWaypointReached = globalWaypointReached(state.position, state.globalWaypoint)
        newGlobalPathReceived = sailbot.newGlobalPathReceived
        localPathIndexOutOfBounds = localPathIndex >= len(localPathLatlons)
        costTooHigh = pathCostThresholdExceeded(localPathSS.getOptimizationObjective(), solutionPathObject)
        pathNotReachGoal = pathDoesNotReachGoal(localPathLatlons, state.globalWaypoint)
        isLocalWaypointReached = localWaypointReached(state.position, localPathLatlons, localPathIndex, referenceLatlon)
        if hasUpwindOrDownwindOnPath or hasObstacleOnPath or isGlobalWaypointReached or newGlobalPathReceived or localPathIndexOutOfBounds or localPathUpdateRequested or costTooHigh or pathNotReachGoal:
            # Log reason for local path update
            rospy.logwarn("Updating Local Path. Reason: hasUpwindOrDownwindOnPath? {}. hasObstacleOnPath? {}. isTimeLimitExceeded? {}. isGlobalWaypointReached? {}. newGlobalPathReceived? {}. localPathIndexOutOfBounds? {}. localPathUpdateRequested? {}. costTooHigh? {}. pathNotReachGoal? {}".format(hasUpwindOrDownwindOnPath, hasObstacleOnPath, isTimeLimitExceeded, isGlobalWaypointReached, newGlobalPathReceived, localPathIndexOutOfBounds, localPathUpdateRequested, costTooHigh, pathNotReachGoal))

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
            localPathSS, solutionPathObject, referenceLatlon = createLocalPathSS(state, plot=True, resetSpeedupDuringPlan=True, speedupBeforePlan=speedup)
            newCost = solutionPathObject.cost(localPathSS.getOptimizationObjective()).value()
            rospy.logerr("New path cost is: {}".format(newCost))
            localPathLatlons = getLocalPathLatlons(solutionPathObject, referenceLatlon)
            localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
            localWaypoint = getLocalWaypointLatLon(localPathLatlons, localPathIndex)
            lastTimePathCreated = time.time()
        elif isLocalWaypointReached or isTimeLimitExceeded:
            # Update current path
            state = sailbot.getCurrentState()
            lengthChange = removePastWaypointsInSolutionPath(localPathSS, solutionPathObject, referenceLatlon, state)
            if lengthChange == 0:
                localPathIndex += 1
            else:
                localPathIndex = 1
            localPathLatlons = getLocalPathLatlons(solutionPathObject, referenceLatlon)
            localWaypoint = getLocalWaypointLatLon(localPathLatlons, localPathIndex)

            # Compare with new path
            _localPathSS, _solutionPathObject, _referenceLatlon = createLocalPathSS(state, plot=True, resetSpeedupDuringPlan=True, speedupBeforePlan=speedup)
            newCost = _solutionPathObject.cost(_localPathSS.getOptimizationObjective()).value()
            rospy.logerr("New path cost is: {}".format(newCost))
            lastTimePathCreated = time.time()
            if newCost < currentCost:
                rospy.logerr("Updating path")
                localPathSS, solutionPathObject, referenceLatlon = _localPathSS, _solutionPathObject, _referenceLatlon
                localPathLatlons = getLocalPathLatlons(solutionPathObject, referenceLatlon)
                localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
                localWaypoint = getLocalWaypointLatLon(localPathLatlons, localPathIndex)
            else:
                rospy.logerr("Not updating path")


        # Publish desiredHeading
        desiredHeadingMsg.headingDegrees = getDesiredHeading(state.position, localWaypoint)
        desiredHeadingPublisher.publish(desiredHeadingMsg)

        # Publish local path
        localPathPublisher.publish(msg.path(localPathLatlons))

        # Update wind direction of localPathSS
        updateWindDirectionInSS(localPathSS, state)
        updateObjectsInSS(localPathSS, referenceLatlon, state)

        # Publish nextLocalWaypoint and nextGlobalWaypoint and path cost
        nextLocalWaypointPublisher.publish(localWaypoint)
        nextGlobalWaypointPublisher.publish(state.globalWaypoint)
        pathCostPublisher.publish(solutionPathObject.cost(localPathSS.getOptimizationObjective()).value())
        pathCostBreakdownPublisher.publish(getPathCostBreakdownString(localPathSS.getOptimizationObjective(), solutionPathObject))

        # If there are any plots, give some time for them to respond to requests, such as closing
        plt.pause(0.001)
        publishRate.sleep()
