#!/usr/bin/env python
import time
from plotting import plot_path
from ompl import util as ou
from ompl import geometric as og
from updated_geometric_planner import plan, Obstacle
from cli import parse_obstacle
import math
from geopy.distance import great_circle
import local_pathfinding.msg as msg


def parse_AIS_msg(AIS_msg):
    obstacles = []
    for ship in AIS_msg.ships:
        obstacles.append(parse_obstacle("{},{},{}".format(ship.lat, ship.lon, ship.speed)))
    return obstacles


def createLocalPath(currentState):
    # Solve the planning problem
    solutions = []
    start = [currentState.currentPosition[0], currentState.currentPosition[1]]
    goal = [currentState.globalWaypoint.lat, currentState.globalWaypoint.lon]
    print("Start: {0}, {1}".format(start[0], start[1]))
    print("Goal: {0}, {1}".format(goal[0], goal[1]))
    # dimensions = [min(start[0], goal[0]), min(start[1], goal[1]), max(start[0], goal[0]), max(start[1], goal[1])]
    extra = 3
    dimensions = [start[0] - extra, start[1] - extra, goal[0] + extra, goal[1] + extra]
    # obstacles = [parse_obstacle("2.5,2.5,1")]
    obstacles = parse_AIS_msg(currentState.AISData)
    windDirection = currentState.wind_direction
    runtime = 3
    for i in range(4):
        solutions.append(plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', windDirection, dimensions, start, goal,
         obstacles))

    solution = min(solutions, key=lambda x: x[0])
    solution_path = solution[2]
    plot_path(solution[1], dimensions, obstacles)
    return solution_path

def isBadPath(state, localPath, localPathIndex):
    # If sailing upwind or downwind, isBad
    if math.fabs(state.wind_direction - getDesiredHeading(state.currentPosition, localPath[localPathIndex])) < math.radians(30) or math.fabs(state.wind_direction - getDesiredHeading(state.currentPosition, localPath[localPathIndex]) - math.radians(180)) < math.radians(30):
        return True

    # Check if path will hit objects
    return False

def globalWaypointReached(currentPosition, globalWaypoint):
    radius = 2
    sailbot = (currentPosition[0], currentPosition[1])
    waypt = (globalWaypoint.lat, globalWaypoint.lon)
    return great_circle(sailbot, waypt) < radius

def localWaypointReached(currentPosition, localPath, localPathIndex):
    radius = 1
    sailbot = (currentPosition[0], currentPosition[1])
    waypt = (localPath[localPathIndex][0], localPath[localPathIndex][1])
    return great_circle(sailbot, waypt) < radius

def timeLimitExceeded(lastTimePathCreated):
    secondsLimit = 5
    return time.time() - lastTimePathCreated > secondsLimit

def getDesiredHeading(currentPosition, localWaypoint):
    return math.atan2(localWaypoint[1] - currentPosition[1], localWaypoint[0] - currentPosition[0])
