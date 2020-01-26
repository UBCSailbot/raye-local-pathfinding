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

def createLocalPathSS(state):
    ou.setLogLevel(ou.LOG_WARN)

    # Get setup parameters from state
    start = [state.position.lat, state.position.lon]
    goal = [state.globalWaypoint.lat, state.globalWaypoint.lon]
    extra = 3   # Extra length to show more in the plot
    dimensions = [min(start[0], goal[0]) - extra, min(start[1], goal[1]) - extra, max(start[0], goal[0]) + extra, max(start[1], goal[1]) + extra]
    obstacles = [parse_obstacle("{},{},{}".format(ship.lat, ship.lon, ship.speed)) for ship in state.AISData.ships]
    windDirection = state.windDirection
    runtime = 1

    # Run the planner multiple times and find the best one
    numRuns = 2
    solutions = []
    print("start: {} {}".format(start[0], start[1]))
    print("goal: {} {}".format(goal[0], goal[1]))
    print("dimensions: {} {} {} {}".format(dimensions[0], dimensions[1], dimensions[2], dimensions[3]))
    for i in range(numRuns):
        solutions.append(plan(runtime, "RRTStar", 'WeightedLengthAndClearanceCombo', windDirection, dimensions, start, goal, obstacles))

    solution = min(solutions, key=lambda x: x.getSolutionPath().cost(x.getOptimizationObjective()).value())

    # Plot
    plot_path(solution.getSolutionPath().printAsMatrix(), dimensions, obstacles)

    return solution

def badPath(state, localPathSS, desiredHeading):
    # If sailing upwind or downwind, isBad
    if math.fabs(state.windDirection - desiredHeading) < math.radians(30) or math.fabs(state.windDirection - desiredHeading - math.radians(180)) < math.radians(30):
        rospy.loginfo_throttle(1, "Sailing upwind/downwind. Wind direction: {}. Desired Heading: {}".format(state.windDirection, desiredHeading))  # Prints every x seconds
        return True

    # Check if path will hit objects
    obstacles = [parse_obstacle("{},{},{}".format(ship.lat, ship.lon, ship.speed)) for ship in state.AISData.ships]
    if not hasNoCollisions(localPathSS, obstacles):
        rospy.loginfo("Going to hit obstacle.")
        return True
    else:
        rospy.loginfo("Not going to hit obstacle.")

    return False

def globalWaypointReached(position, globalWaypoint):
    radius = 2
    sailbot = (position.lat, position.lon)
    waypt = (globalWaypoint.lat, globalWaypoint.lon)
    return great_circle(sailbot, waypt) < radius

def localWaypointReached(position, localPath, localPathIndex):
    radius = 1
    sailbot = (position.lat, position.lon)
    waypt = (localPath[localPathIndex].getX(), localPath[localPathIndex].getY())
    return great_circle(sailbot, waypt) < radius

def timeLimitExceeded(lastTimePathCreated):
    secondsLimit = 5000
    return time.time() - lastTimePathCreated > secondsLimit

def setLocalWaypointLatLon(localWaypointLatLon, localWaypoint):
    localWaypointLatLon.lat = localWaypoint.getX()
    localWaypointLatLon.lon = localWaypoint.getY()
    return localWaypointLatLon

# this will give initial bearing on a great-circle path
#if we keep local waypoints close enough to each other it approx the final bearing
def getDesiredHeading(position, localWaypoint):
    term1 = math.sin(localWaypoint.lon - position.lon) * math.cos(localWaypoint.lat)
    term2 = math.cos(position.lat) * math.sin(localWaypoint.lat) - math.sin(position.lat) * math.cos(localWaypoint.lat)*math.cos(localWaypoint.lon - position.lon)
    bearing = math.degrees(math.atan2(term1, term2))
    
    #this changes the bearing into a heading assuming 0 degrees pointing east for heading measurements
    heading = bearing - 90
    if (heading < 0):
        heading += 360
    return heading
