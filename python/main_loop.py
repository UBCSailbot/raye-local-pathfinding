#!/usr/bin/env python

import rospy
import local_pathfinding.msg as msg
from Sailbot import *
from utilities import *
import time

if __name__ == '__main__':
    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = Sailbot(nodeName='local_pathfinding')

    # Create ros publisher for the desired heading for the controller
    desiredHeadingPublisher = rospy.Publisher('MOCK_desired_heading', msg.desired_heading, queue_size=4)
    desiredHeadingMsg = msg.desired_heading()

    # Create other publishers
    localPathPublisher = rospy.Publisher('MOCK_local_path', msg.path, queue_size=4)
    nextLocalWaypointPublisher = rospy.Publisher('MOCK_next_local_waypoint', msg.latlon, queue_size=4)
    nextGlobalWaypointPublisher = rospy.Publisher('MOCK_next_global_waypoint', msg.latlon, queue_size=4)
    publishRate = rospy.Rate(1) # Hz

    # Wait until first global path is received
    rospy.loginfo("Waiting for sailbot to receive newGlobalPath")
    while not sailbot.newGlobalPathReceived:
        rospy.loginfo("Still waiting...")
        time.sleep(1)

    # Create first path and track time of updates
    state = sailbot.getCurrentState()
    localPathSS, referenceLatlon = createLocalPathSS(state)
    localPath = getLocalPath(localPathSS, referenceLatlon)
    localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
    localWaypoint = getLocalWaypointLatLon(localPath, localPathIndex)
    lastTimePathCreated = time.time()

    while not rospy.is_shutdown():
        rospy.loginfo("sailbot.globalPathIndex: {}".format(sailbot.globalPathIndex))
        rospy.loginfo("localPathIndex: {}".format(localPathIndex))
        state = sailbot.getCurrentState()

        # Generate new local path if needed
        isBad = badPath(state, localPathSS, referenceLatlon, desiredHeadingMsg.heading)
        isTimeLimitExceeded = timeLimitExceeded(lastTimePathCreated)
        isGlobalWaypointReached = globalWaypointReached(state.position, state.globalWaypoint)
        newGlobalPathReceived = sailbot.newGlobalPathReceived
        localPathIndexOutOfBounds = localPathIndex >= len(localPath)
        if isBad or isTimeLimitExceeded or isGlobalWaypointReached or newGlobalPathReceived or localPathIndexOutOfBounds:
            # Log reason for local path update
            rospy.loginfo("Updating Local Path. Reason: isBad? {}. isTimeLimitExceeded? {}. isGlobalWaypointReached? {}. newGlobalPathReceived? {}. localPathIndexOutOfBounds? {}.".format(isBad, isTimeLimitExceeded, isGlobalWaypointReached, newGlobalPathReceived, localPathIndexOutOfBounds))

            # Reset saiblot newGlobalPathReceived boolean
            if newGlobalPathReceived:
                sailbot.newGlobalPathReceived = False

            # If globalWaypoint reached, increment the index
            if isGlobalWaypointReached:
                rospy.loginfo("Global waypoint reached")
                sailbot.globalPathIndex += 1

            # Update local path
            state = sailbot.getCurrentState()
            localPathSS, referenceLatlon = createLocalPathSS(state)
            localPath = getLocalPath(localPathSS, referenceLatlon)
            localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
            localWaypoint = getLocalWaypointLatLon(localPath, localPathIndex)
            lastTimePathCreated = time.time()

        # If localWaypoint met, increment the index
        elif localWaypointReached(state.position, localPath, localPathIndex):
            rospy.loginfo("Local waypoint reached")
            localPathIndex += 1
            localWaypoint = getLocalWaypointLatLon(localPath, localPathIndex)

        # Publish desiredHeading
        desiredHeadingMsg.heading = getDesiredHeading(state.position, localWaypoint)
        rospy.loginfo_throttle(10, "desiredHeadingMsg: {}".format(desiredHeadingMsg.heading))  # Prints every x seconds
        desiredHeadingPublisher.publish(desiredHeadingMsg)
        publishRate.sleep()

        # Publish local path
        localPathPublisher.publish(msg.path(localPath))

        # Publish nextLocalWaypoint and nextGlobalWaypoint
        nextLocalWaypointPublisher.publish(localWaypoint)
        nextGlobalWaypointPublisher.publish(state.globalWaypoint)
