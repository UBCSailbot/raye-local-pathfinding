#!/usr/bin/env python
from ompl import util as ou
import rospy
import time
from plotting import plot_path
from ompl import util as ou
from ompl import geometric as og
from updated_geometric_planner import plan, Obstacle, hasNoCollisions
from cli import parse_obstacle
import math
from geopy.distance import great_circle
import local_pathfinding.msg as msg
import planner_helpers as ph

import math
import sys

from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from math import sqrt

import planner_helpers as ph

import math
import sys
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from math import sqrt

import planner_helpers as ph

def createLocalPath(state):
    ou.setLogLevel(ou.LOG_WARN)
    # Get setup parameters from state
    start = [state.position[0], state.position[1]]
    goal = [state.globalWaypoint.lat, state.globalWaypoint.lon]
    extra = 3   # Extra dimensions to show more in the plot
    dimensions = [start[0] - extra, start[1] - extra, goal[0] + extra, goal[1] + extra]
    obstacles = [parse_obstacle("{},{},{}".format(ship.lat, ship.lon, ship.speed)) for ship in state.AISData.ships]
    windDirection = state.windDirection
    runtime = 1

#     # Run the planner multiple times and find the best one
#     solutions = []
#     for i in range(4):
#         solutions.append(plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', windDirection, dimensions, start, goal,
#          obstacles))
# 
#     solution = min(solutions, key=lambda x: x[0])
#     solution_path = solution[2]
# 
#     print("About to start loop")
#     for state in solution[3]:
#         print("In loop")
#         print("{}{}{}".format(state.getX(), state.getY(), state.getYaw()))
#     print("After loop")
#     # Plot
#     plot_path(solution[1], dimensions, obstacles)
#     return (solution_path, solution[3])

    ss = plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', windDirection, dimensions, start, goal, obstacles)
    states = ss.getSolutionPath().getStates()
    print("****")
    print("type(states): {}".format(type(states)))
    print("print(states): {}".format(states))

    print("About to start loop 2")
    for state in states:
        print("In loop")
        print("{}, {}, {}".format(state.getX(), state.getY(), state.getYaw()))
    print("After loop")

    return ss

def badPath(state, localPath, localPathIndex, myVector):
    # If sailing upwind or downwind, isBad
#     desiredHeading = getDesiredHeading(state.position, localPath[localPathIndex])
#     if math.fabs(state.windDirection - desiredHeading) < math.radians(30) or math.fabs(state.windDirection - desiredHeading - math.radians(180)) < math.radians(30):
#         rospy.loginfo_throttle(1, "Sailing upwind/downwind. Wind direction: {}. Desired Heading: {}".format(state.windDirection, desiredHeading))  # Prints every x seconds
#         return True

    # Check if path will hit objects
    start = [state.position[0], state.position[1]]
    goal = [state.globalWaypoint.lat, state.globalWaypoint.lon]
    extra = 3   # Extra dimensions to show more in the plot
    dimensions = [start[0] - extra, start[1] - extra, goal[0] + extra, goal[1] + extra]
    obstacles = [parse_obstacle("{},{},{}".format(ship.lat, ship.lon, ship.speed)) for ship in state.AISData.ships]
    print("OK 1")
    if hasNoCollisions(localPath, obstacles, dimensions, myVector):
        rospy.loginfo_throttle(1, "Going to hit obstacle.")  # Prints every x seconds
        return True

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
