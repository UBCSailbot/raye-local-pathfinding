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


def createLocalPath(state):
    # Get setup parameters from state
    start = [state.position[0], state.position[1]]
    goal = [state.globalWaypoint.lat, state.globalWaypoint.lon]
    print("Start: {0}, {1}".format(start[0], start[1]))
    print("Goal: {0}, {1}".format(goal[0], goal[1]))
    extra = 3   # Extra dimensions to show more in the plot
    dimensions = [start[0] - extra, start[1] - extra, goal[0] + extra, goal[1] + extra]
    obstacles = [parse_obstacle("{},{},{}".format(ship.lat, ship.lon, ship.speed)) for ship in state.AISData.ships]
    windDirection = state.windDirection
    runtime = 3

    # Run the planner multiple times and find the best one
    solutions = []
    for i in range(4):
        solutions.append(plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', windDirection, dimensions, start, goal,
         obstacles))
    solution = min(solutions, key=lambda x: x[0])
    solution_path = solution[2]

    # Plot
    plot_path(solution[1], dimensions, obstacles)
    return solution_path

def badPath(state, localPath, localPathIndex):
    # If sailing upwind or downwind, isBad
    if math.fabs(state.windDirection - getDesiredHeading(state.position, localPath[localPathIndex])) < math.radians(30) or math.fabs(state.windDirection - getDesiredHeading(state.position, localPath[localPathIndex]) - math.radians(180)) < math.radians(30):
        return True

    # Check if path will hit objects
    return False

def globalWaypointReached(position, globalWaypoint):
    radius = 2
    sailbot = (position[0], position[1])
    waypt = (globalWaypoint.lat, globalWaypoint.lon)
    return great_circle(sailbot, waypt) < radius

def localWaypointReached(position, localPath, localPathIndex):
    radius = 1
    sailbot = (position[0], position[1])
    waypt = (localPath[localPathIndex][0], localPath[localPathIndex][1])
    return great_circle(sailbot, waypt) < radius

def timeLimitExceeded(lastTimePathCreated):
    secondsLimit = 5
    return time.time() - lastTimePathCreated > secondsLimit

def getDesiredHeading(position, localWaypoint):
    return math.atan2(localWaypoint[1] - position[1], localWaypoint[0] - position[0])
