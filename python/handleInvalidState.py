#!/usr/bin/env python

import rospy
import math
import planner_helpers as ph
import sailbot_msg.msg as msg
import obstacles as obs
import utilities as utils

MOVE_GOAL_WAYPOINT_STEP_SIZE_KM = 5


def getValidStateMoveToSafetyIfNeeded(sailbot, desiredHeadingPublisher):
    '''Checks if the sailbot's start state is valid (nearby obstacle). If not, send desired heading to safety till valid

    Args:
       sailbot (Sailbot): Sailbot object with which current state will be calculated
       desiredHeadingPublisher (rospy.Publisher:msg.heading) publisher for desired heading of sailbot to move to safety

    Returns:
       BoatState of the sailbot that has a valid start state
    '''
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
    '''Checks if there are any obstacles projected to be on the state's starting position

    Args:
       state (BoatState): State of the sailbot and other boats

    Returns:
       ObstacleInterface object that is at the start state causing it to be invalid, else returns None
       If there are multiple obstacles on start, this just returns one of them
    '''
    referenceLatlon = state.globalWaypoint
    obstacles = obs.getObstacles(state, referenceLatlon)
    for obstacle in obstacles:
        xy = utils.latlonToXY(state.position, referenceLatlon)
        if not obstacle.isValid(xy):
            return obstacle
    return None


def generateSafeHeadingDegrees(state, invalidStartObstacle):
    '''Finds a heading that the sailbot can safely follow to get away from a closeby AIS boat
    It returns a heading perpendicular to the heading of the nearby boat to ensure we get out of its way.
    Since there are two directions, we choose either one, as long as it is not upwind

    Args:
       state (BoatState): State of the sailbot and other boats
       invalidStartObstacle (ObstacleInterface): obstacle that is causing the sailbot start state to be invalid

    Returns:
       float heading in degrees (0=East, 90=North) that sailbot should follow to get away from the AIS boat
    '''
    rospy.logwarn("Generating heading to safety...")
    obstacleHeadingDegrees = invalidStartObstacle.aisShip.headingDegrees
    potentialHeadingsDegrees = [obstacleHeadingDegrees + 90, obstacleHeadingDegrees - 90]

    for headingDegrees in potentialHeadingsDegrees:
        _, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph,
            state.headingDegrees)
        if not ph.isUpwind(math.radians(globalWindDirectionDegrees), math.radians(headingDegrees)):
            rospy.logwarn("Found heading to safety: {}".format(headingDegrees))
            return headingDegrees

    # if all else fails, go downwind
    rospy.logwarn("Couldn't find conventional heading to safety, using downwind direction: {}"
                  .format(globalWindDirectionDegrees))
    return globalWindDirectionDegrees


def checkGoalValidity(state, goalLatlon=None):
    '''Checks if the goal of the given state is valid (no obstacle there)

    Args:
       state (BoatState): State of the sailbot and other boats
       goalLatlon (latlon): Location to check the validity of. If is None, goal is the current global waypoint

    Returns:
       bool True iff there is no obstacle projected to be on the state's goal waypoint
    '''
    if goalLatlon is None:
        goalLatlon = state.globalWaypoint

    referenceLatlon = goalLatlon
    obstacles = obs.getObstacles(state, referenceLatlon)
    for obstacle in obstacles:
        goalXY = utils.latlonToXY(goalLatlon, referenceLatlon)
        if not obstacle.isValid(goalXY):
            return False
    return True


def moveGlobalWaypointUntilValid(state, isLast, otherWaypoint):
    '''Draws straight line between other and current global waypoint, then moves goalLatlon away from sailbot along this
    line until it is valid. Other waypoint is the previous global waypoint if isLast, else the next global waypoint

    Args:
       state (BoatState): State of the sailbot and other boats
       isLast (boolean): True if current global waypoint is the last one, else False
       otherWaypoint (latlon): Previous global waypoint if isLast, else the next global waypoint

    Returns:
        goalLatlon (latlon) such that checkGoalValidity(state, newGoal) is true
    '''
    referenceLatlon = state.globalWaypoint
    goalLatlon = state.globalWaypoint
    goalX, goalY = utils.latlonToXY(goalLatlon, referenceLatlon)
    otherX, otherY = utils.latlonToXY(otherWaypoint, referenceLatlon)

    while not checkGoalValidity(state, goalLatlon):
        deltaX = goalX - otherX if isLast else otherX - goalX
        deltaY = goalY - otherY if isLast else otherY - goalY
        dist = math.sqrt(deltaX**2 + deltaY**2)

        goalX += MOVE_GOAL_WAYPOINT_STEP_SIZE_KM * deltaX / dist
        goalY += MOVE_GOAL_WAYPOINT_STEP_SIZE_KM * deltaY / dist
        goalLatlon = utils.XYToLatlon((goalX, goalY), referenceLatlon)

    return goalLatlon
