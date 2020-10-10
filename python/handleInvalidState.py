#!/usr/bin/env python

import rospy
import math
import planner_helpers as ph
import local_pathfinding.msg as msg
import obstacles as obs
import utilities as utils


def getValidStateMoveToSafetyIfNeeded(sailbot, desiredHeadingPublisher):
    state = sailbot.getCurrentState()
    obstacleOnStartPosition = obstacleOnStart(state)
    startValid = (obstacleOnStartPosition is None)

    if not startValid:
        while not startValid and not rospy.is_shutdown():
            headingToSafetyDegrees = generateSafeHeadingDegrees(state, obstacleOnStartPosition)
            rospy.logwarn("INVALID START STATE! Publishing Heading to safety: {}".format(headingToSafetyDegrees))
            desiredHeadingPublisher.publish(msg.heading(headingToSafetyDegrees))
            state = sailbot.getCurrentState()
            obstacleOnStartPosition = obstacleOnStart(state)
            startValid = (obstacleOnStartPosition is None)
        rospy.loginfo("Start state OK")
    return state


def obstacleOnStart(state):
    referenceLatlon = state.globalWaypoint
    obstacles = obs.getObstacles(state, referenceLatlon)
    for obstacle in obstacles:
        xy = utils.latlonToXY(state.position, referenceLatlon)
        if not obstacle.isValid(xy):
            return obstacle
    return None


def generateSafeHeadingDegrees(state, invalidStartObstacle):
    rospy.logwarn("Generating heading to safety...")
    obstacleHeadingDegrees = invalidStartObstacle.aisShip.headingDegrees
    potentialHeadingsDegrees = [obstacleHeadingDegrees + 90, obstacleHeadingDegrees - 90]

    for headingDegrees in potentialHeadingsDegrees:
        _, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph,
            state.headingDegrees)
        if not ph.isUpwind(math.radians(globalWindDirectionDegrees), math.radians(headingDegrees)):
            rospy.logwarn("Found heading to safety: {}".headingDegrees)
            return headingDegrees

    # if all else fails, go downwind
    rospy.logwarn("Couldn't find conventional heading to safety, using downwind direction: {}"
                  .format(globalWindDirectionDegrees))
    return globalWindDirectionDegrees


def checkGoalValidity(state):
    referenceLatlon = state.globalWaypoint
    obstacles = obs.getObstacles(state, referenceLatlon)
    for obstacle in obstacles:
        goalXY = utils.latlonToXY(state.globalWaypoint, referenceLatlon)
        if not obstacle.isValid(goalXY):
            return False
    return True
