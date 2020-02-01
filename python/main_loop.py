#!/usr/bin/env python

import rospy

import local_pathfinding.msg as msg
import Sailbot
from Sailbot import *

import utilities
from utilities import *

import time
#############
import math
import sys

from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from math import sqrt

import planner_helpers as ph


if __name__ == '__main__':
    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = Sailbot()

    # Create ros publisher for the desired heading for the controller
    desiredHeadingPublisher = rospy.Publisher('MOCK_desired_heading', msg.desired_heading)
    localPathPublisher = rospy.Publisher('MOCK_local_path', msg.path)
    nextLocalWaypointPublisher = rospy.Publisher('MOCK_next_local_waypoint', msg.latlon)
    nextGlobalWaypointPublisher = rospy.Publisher('MOCK_next_global_waypoint', msg.latlon)
    publishRate = rospy.Rate(1) # Hz
    desiredHeadingMsg = msg.desired_heading()

    # Create ros publisher for the local path, only for testing
    localPathPublisher = rospy.Publisher('MOCK_local_path', msg.path)

    # Create first path and track time of updates
    state = sailbot.getCurrentState()
    localPathSS = createLocalPathSS(state)
    localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
    localWaypoint = getLocalWaypointLatLon(localPathSS, localPathIndex)

    lastTimePathCreated = time.time()

    while not rospy.is_shutdown():
        print("sailbot.globalPathIndex: {}".format(sailbot.globalPathIndex))
        print("localPathIndex: {}".format(localPathIndex))
        state = sailbot.getCurrentState()

        # Generate new local path if needed.
        isBad = badPath(state, localPathSS, desiredHeadingMsg.heading)
        isTimeLimitExceeded = timeLimitExceeded(lastTimePathCreated)
        isGlobalWaypointReached = globalWaypointReached(state.position, state.globalWaypoint)
        if isBad or isTimeLimitExceeded or isGlobalWaypointReached:
            rospy.loginfo("Updating localPath")
            rospy.loginfo("isBad? {}. isTimeLimitExceeded? {}. isGlobalWaypointReached? {}.".format(isBad, isTimeLimitExceeded, isGlobalWaypointReached))

            # If globalWaypoint met, increment the index
            if isGlobalWaypointReached:
                rospy.loginfo("Global waypoint reached")
                sailbot.globalPathIndex += 1

            # Update local path
            localPathSS = createLocalPathSS(state)
            localPathIndex = 1  # First waypoint is the start point, so second waypoint is the next local waypoint
            localWaypoint = getLocalWaypointLatLon(localPathSS, localPathIndex)
            lastTimePathCreated = time.time()

        # If localWaypoint met, increment the index
        elif localWaypointReached(state.position, localPathSS.getSolutionPath().getStates(), localPathIndex):
            rospy.loginfo("Local waypoint reached")
            localPathIndex += 1
            localWaypoint = getLocalWaypointLatLon(localPathSS, localPathIndex)

        # Publish desiredHeading
        desiredHeadingMsg.heading = getDesiredHeading(state.position, localWaypoint)
        rospy.loginfo_throttle(1, "desiredHeadingMsg: {}".format(desiredHeadingMsg.heading))  # Prints every x seconds
        desiredHeadingPublisher.publish(desiredHeadingMsg)
        publishRate.sleep()

        # Publish local path
        path = []
        for state in localPathSS.getSolutionPath().getStates():
            latlon = msg.latlon()
            latlon.lat = state.getY()
            latlon.lon = state.getX()
            path.append(latlon)

        localPathPublisher.publish(msg.path(path))

        # Publish nextLocalWaypoint and nextGlobalWaypoint
        nextLocalWaypointPublisher.publish(localWaypoint)
        nextGlobalWaypointPublisher.publish(sailbot.getCurrentState().globalWaypoint)
