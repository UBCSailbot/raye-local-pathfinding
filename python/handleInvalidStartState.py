#!/usr/bin/env python

import rospy
import math
import planner_helpers as ph
import local_pathfinding.msg as msg
import obstacles as obs
import utilities as utils


def getValidState(sailbot, desiredHeadingPublisher):
    state = sailbot.getCurrentState()
    obstacleOnStartPosition = obstacleOnStart(state)
    startValid = (obstacleOnStartPosition is None)

    while not startValid and not rospy.is_shutdown():
        headingToSafety = generateSafeHeading(state, obstacleOnStartPosition)
        rospy.logwarn("INVALID START STATE! Publishing Heading: {}".format(headingToSafety))
        desiredHeadingPublisher.publish(msg.heading(headingToSafety))
        state = sailbot.getCurrentState()
        obstacleOnStartPosition = obstacleOnStart(state)
        startValid = (obstacleOnStartPosition is None)
    rospy.logwarn("Start state OK")
    return state


def obstacleOnStart(state):
    referenceLatlon = state.globalWaypoint
    obstacles = obs.getObstacles(state, referenceLatlon)
    for obstacle in obstacles:
        xy = utils.latlonToXY(state.position, referenceLatlon)
        if not obstacle.isValid(xy):
            return obstacle
    return None


def generateSafeHeading(state, invalidStartObstacle):
    obstacleHeadingRad = math.radians(invalidStartObstacle.aisShip.headingDegrees)
    potentialHeadingsRad = []
    potentialHeadingsRad.append(obstacleHeadingRad + math.pi * 0.5)
    potentialHeadingsRad.append(obstacleHeadingRad - math.pi * 0.5)

    for headingRad in potentialHeadingsRad:
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph,
            state.headingDegrees)
        rospy.logwarn("Global Wind Direction: {}".format(globalWindDirectionDegrees))
        if not ph.isUpwind(math.radians(globalWindDirectionDegrees), headingRad):
            return math.degrees(headingRad)
    # if all else fails, go downwind
    return globalWindDirectionDegrees
