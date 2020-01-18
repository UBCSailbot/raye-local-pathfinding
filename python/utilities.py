#!/usr/bin/env python
import time
from Sailbot import GPSCoordinates

def getCurrentLandAndBorderData(currentState):
    return None

def createNewPath(currentState, currentLandAndBorderData):
    return [ [currentState.currentPosition.latitude, currentState.currentPosition.longitude], [currentState.currentPosition.latitude + 1, currentState.currentPosition.longitude + 1] ]

def nextGlobalWaypointReached(currentState):
    distanceThreshold = 0.5
    return distance(currentState.currentPosition, currentState.globalWaypoint) < distanceThreshold

def isBadPath(currentState, currentPath, currentLandAndBorderData):
    return None

def nextLocalWaypointReached(currentState, nextLocalWaypointMsg):
    distanceThreshold = 0.5
    return distance(currentState.currentPosition, GPSCoordinates(nextLocalWaypointMsg.x, nextLocalWaypointMsg.y)) < distanceThreshold

def timeLimitExceeded(lastTimePathCreated):
    secondsLimit = 5
    return time.time() - lastTimePathCreated > secondsLimit

def distance(gpsCoord1, gpsCoord2):
    return ((gpsCoord1.latitude - gpsCoord2.latitude)**2 + (gpsCoord1.longitude - gpsCoord2.longitude)**2)**0.5
