#!/usr/bin/env python
import time
from plotting import plot_path
from ompl import util as ou
from ompl import geometric as og
from updated_geometric_planner import plan, Obstacle
from cli import parse_obstacle
import math


def createLocalPath(currentState):
    # Solve the planning problem
    solutions = []
    dimensions = [0,0,10,10]
    obstacles = [parse_obstacle("2.5,2.5,1")]
    runtime = 0.1
    for i in range(4):
        solutions.append(plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', 0, dimensions, [0,0], [10,10],
         obstacles))

    solution = min(solutions, key=lambda x: x[0])
    solution_path = solution[2]
    plot_path(solution[1], dimensions, obstacles)
    return solution_path

def nextGlobalWaypointReached(currentState):
    distanceThreshold = 0.5
    return distance(currentState.currentPosition, currentState.globalWaypoint) < distanceThreshold

def isBadPath(state, localPath, localPathIndex):
    # If sailing upwind or downwind, isBad
    if abs(state.wind_direction - getDesiredHeading(state.currentPosition, localPath[localPathIndex])):
        return True

    # Check if path will hit objects
    return False

def globalWaypointReached(currentPosition, globalWaypoint):
    return False

def nextLocalWaypointReached(currentState, nextLocalWaypointMsg):
    distanceThreshold = 0.5
    return distance(currentState.currentPosition, GPSCoordinates(nextLocalWaypointMsg.x, nextLocalWaypointMsg.y)) < distanceThreshold

def timeLimitExceeded(lastTimePathCreated):
    secondsLimit = 5
    return time.time() - lastTimePathCreated > secondsLimit

def getDesiredHeading(currentPosition, localWaypoint):
    return math.radians(180)
