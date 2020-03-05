#!/usr/bin/env python

import sys
import rospy
import local_pathfinding.msg as msg
from Sailbot import *
from utilities import *
import time

# Constants
MAIN_LOOP_PERIOD_SECONDS = 1.0

if __name__ == '__main__':
    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = Sailbot(nodeName='local_pathfinding')

    # Create ros publisher for the desired heading for the controller
    desiredHeadingPublisher = rospy.Publisher('desiredHeading', msg.heading, queue_size=4)
    desiredHeadingMsg = msg.heading()

    # Create other publishers
    localPathPublisher = rospy.Publisher('localPath', msg.path, queue_size=4)
    nextLocalWaypointPublisher = rospy.Publisher('nextLocalWaypoint', msg.latlon, queue_size=4)
    nextGlobalWaypointPublisher = rospy.Publisher('nextGlobalWaypoint', msg.latlon, queue_size=4)
    publishRate = rospy.Rate(1.0 / MAIN_LOOP_PERIOD_SECONDS)

    # Get speedup parameter
    speedup = rospy.get_param('speedup', default=1.0)

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
    localPathSS, referenceLatlon = createLocalPathSS(state, plot=True)
    printCostBreakdown(localPathSS)

    localPath = getLocalPath(localPathSS, referenceLatlon)
    localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
    localWaypoint = getLocalWaypointLatLon(localPath, localPathIndex)
    lastTimePathCreated = time.time()
    sailbot.newGlobalPathReceived = False

    while not rospy.is_shutdown():
        rospy.loginfo("sailbot.globalPathIndex: {}".format(sailbot.globalPathIndex))
        rospy.loginfo("localPathIndex: {}".format(localPathIndex))
        rospy.loginfo("desiredHeadingMsg: {}".format(desiredHeadingMsg.headingDegrees))
        state = sailbot.getCurrentState()

        # Publish desiredHeading
        desiredHeadingMsg.headingDegrees = getDesiredHeading(state.position, localWaypoint)
        desiredHeadingPublisher.publish(desiredHeadingMsg)

        # Publish local path
        localPathPublisher.publish(msg.path(localPath))

        # Publish nextLocalWaypoint and nextGlobalWaypoint
        nextLocalWaypointPublisher.publish(localWaypoint)
        nextGlobalWaypointPublisher.publish(state.globalWaypoint)

        # If there are any plots, give some time for them to respond to requests, such as closing
        plt.pause(0.001)

        # Generate new local path if needed
        hasUpwindOrDownwindOnPath = upwindOrDownwindOnPath(state, localPathIndex, localPathSS, referenceLatlon, NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND)
        hasObstacleOnPath = obstacleOnPath(state, localPathIndex, localPathSS, referenceLatlon, NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES)
        isTimeLimitExceeded = timeLimitExceeded(lastTimePathCreated, speedup)
        isGlobalWaypointReached = globalWaypointReached(state.position, state.globalWaypoint)
        newGlobalPathReceived = sailbot.newGlobalPathReceived
        localPathIndexOutOfBounds = localPathIndex >= len(localPath)
        if hasUpwindOrDownwindOnPath or hasObstacleOnPath or isTimeLimitExceeded or isGlobalWaypointReached or newGlobalPathReceived or localPathIndexOutOfBounds:
            # Log reason for local path update
            rospy.loginfo("Updating Local Path. Reason: hasUpwindOrDownwindOnPath? {}. hasObstacleOnPath? {}. isTimeLimitExceeded? {}. isGlobalWaypointReached? {}. newGlobalPathReceived? {}. localPathIndexOutOfBounds? {}.".format(hasUpwindOrDownwindOnPath, hasObstacleOnPath, isTimeLimitExceeded, isGlobalWaypointReached, newGlobalPathReceived, localPathIndexOutOfBounds))

            # Reset saiblot newGlobalPathReceived boolean
            if newGlobalPathReceived:
                sailbot.newGlobalPathReceived = False

            # If globalWaypoint reached, increment the index
            if isGlobalWaypointReached:
                rospy.loginfo("Global waypoint reached")
                sailbot.globalPathIndex += 1

            # Update local path
            state = sailbot.getCurrentState()
            localPathSS, referenceLatlon = createLocalPathSS(state, plot=True)
            printCostBreakdown(localPathSS)
            localPath = getLocalPath(localPathSS, referenceLatlon)
            localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
            localWaypoint = getLocalWaypointLatLon(localPath, localPathIndex)
            lastTimePathCreated = time.time()

        # If localWaypoint met, increment the index
        elif localWaypointReached(state.position, localPath, localPathIndex, referenceLatlon):
            rospy.loginfo("Local waypoint reached")
            localPathIndex += 1
            localWaypoint = getLocalWaypointLatLon(localPath, localPathIndex)

        publishRate.sleep()
