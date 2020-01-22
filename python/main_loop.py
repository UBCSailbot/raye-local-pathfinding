#!/usr/bin/env python

import rospy

import local_pathfinding.msg as msg
import Sailbot
from Sailbot import *

import utilities
from utilities import *

import time

if __name__ == '__main__':
    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = Sailbot()

    # Create ros publisher for the next local waypoint for the controller
    desiredHeadingPublisher = rospy.Publisher('MOCK_desired_heading', msg.desired_heading)
    publishRate = rospy.Rate(10) # Hz
    desiredHeadingMsg = msg.desired_heading()

    # Create first path and track time of updates
    state = sailbot.getCurrentState()
    localPath = createLocalPath(state)
    localPathIndex = 0
    lastTimePathCreated = time.time()

    while not rospy.is_shutdown():
        state = sailbot.getCurrentState()

        # Generate new local path if needed.
        isBad = isBadPath(state, localPath)
        isTimeLimitExceeded = timeLimitExceeded(lastTimePathCreated)
        isGlobalWaypointReached = globalWaypointReached(state.currentPosition, state.globalWaypoint)
        if isBad or isTimeLimitExceeded or isGlobalWaypointReached:
            rospy.loginfo("Updating localPath")
            rospy.loginfo("isBad? {}. isTimeLimitExceeded? {}. isGlobalWaypointReached? {}.".format(isBad, isTimeLimitExceeded, isGlobalWaypointReached))

            # If globalWaypoint met, increment the index
            if isGlobalWaypointReached:
                sailbot.globalPathIndex += 1

            # Update local path
            localPath = createLocalPath(state)
            localPathIndex = 0
            lastTimePathCreated = time.time()
        # If localWaypoint met, increment the index
        elif localWaypointReached(state.currentPosition, localPath, localPathIndex):
            localPathIndex += 1

        # Publish desiredHeading
        desiredHeadingMsg.heading = getDesiredHeading(state.currentPosition, localPath[localPathIndex])
        rospy.loginfo_throttle(1, "desiredHeadingMsg: {}".format(desiredHeadingMsg.heading))  # Prints every x seconds
        desiredHeadingPublisher.publish(desiredHeadingMsg)
        publishRate.sleep()
