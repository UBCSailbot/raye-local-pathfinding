import math
import sys
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from math import sqrt

import planner_helpers as ph
import matplotlib.pyplot as plt
from matplotlib import patches

#class Obstacle:
#    def __init__(self, x, y, radius):
#        self.x = x
#        self.y = y
#        self.radius = radius
#    def __str__(self):
#        return str((self.x, self.y, self.radius))
#
class Obstacle:
    def __init__(self, A, B, C):
        self.A = A 
        self.B = B
        self.C = C 

    def __str__(self):
        return str((self.A, self.B, self.C))
       

def absolute_distance_between_angles(angle1, angle2):
    fabs = math.fabs(math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2)))
    return fabs


def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


def hasObstacleOnPath(positionXY, nextLocalWaypointIndex, localPathSS, obstacles):
    # Set the objects used to check which states in the space are valid
    validity_checker = ph.ValidityChecker(localPathSS.getSpaceInformation(), obstacles)
    localPathSS.setStateValidityChecker(validity_checker)

    # Setup for obstacle-on-path checking
    localPathSS.setup()
    stateSpace = localPathSS.getStateSpace()
    solutionPath = localPathSS.getSolutionPath()
    spaceInformation = localPathSS.getSpaceInformation()

    # Handle strange cases with less than 2 states
    if len(solutionPath.getStates()) <= 1:
        print("WARNING: len(solutionPath.getStates()) = {}. Expected >1.".format(len(solutionPath.getStates())))
        if len(solutionPath.getStates()) == 0:
            return False
        else:
            hasObstacle = (not spaceInformation.isValid(solutionPath.getState(0)))
            return hasObstacle

    # Get the relevant states (ignore past states and use current position as first state)
    relevantStates = []
    firstState = spaceInformation.allocState()
    firstState.setXY(positionXY[0], positionXY[1])
    relevantStates.append(firstState)
    for stateIndex in range(nextLocalWaypointIndex, len(solutionPath.getStates())):
        relevantStates.append(solutionPath.getState(stateIndex))

    # Interpolate between states and check for validity
    for stateIndex in range(1, len(relevantStates)):
        # Check in between these points
        prevState = relevantStates[stateIndex - 1]
        nextState = relevantStates[stateIndex]
        interpolatedState = spaceInformation.allocState()

        # Setup checking resolution
        resolution = spaceInformation.getStateValidityCheckingResolution()
        numPoints = int(1 / resolution)

        # Loop so that each fraction is in [0, 1], with bounds inclusive so interpolation checks both the first and last point
        for i in range(numPoints + 1):
            fraction = i * resolution
            stateSpace.interpolate(prevState, nextState, fraction, interpolatedState)
            hasObstacle = (not spaceInformation.isValid(interpolatedState))
            if hasObstacle:
                return True

    '''Uncomment to visualize the obstacles, relevant states, and all states
    ax = plt.gca()
    for obstacle in obstacles:
        # print("Obstacle {}, {}, {}".format(obstacle.x, obstacle.y, obstacle.radius))
        c = plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius)
        c.set_color('r')
        ax.add_patch(c)
    for s in localPathSS.getSolutionPath().getStates():
        # print("SolutionPath State {}, {}".format(s.getX(), s.getY()))
        c = plt.Circle((s.getX(), s.getY()), radius=0.2)
        c.set_color('g')
        ax.add_patch(c)
    for s in relevantStates:
        # print("Relevant State {}, {}".format(s.getX(), s.getY()))
        c = plt.Circle((s.getX(), s.getY()), radius=0.1)
        c.set_color('b')
        ax.add_patch(c)

    c = plt.Circle((positionXY[0], positionXY[1]), radius=0.05)
    c.set_color('m')
    ax.add_patch(c)
    plt.show()
    plt.cla()
    '''

    return False

def plan(run_time, planner_type, objective_type, wind_direction_degrees, dimensions, start_pos, goal_pos, obstacles):
    # Construct the robot state space in which we're planning
    space = ob.SE2StateSpace()

    # Create bounds on the position
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, dimensions[0])
    bounds.setLow(1, dimensions[1])
    bounds.setHigh(0, dimensions[2])
    bounds.setHigh(1, dimensions[3])
    space.setBounds(bounds)

    # Define a simple setup class
    ss = og.SimpleSetup(space)

    # Construct a space information instance for this state space
    si = ss.getSpaceInformation()
    # Set resolution of state validity checking, which is fraction of space's extent (Default is 0.01)
    si.setStateValidityCheckingResolution(0.05)

    # Set the objects used to check which states in the space are valid
    validity_checker = ph.ValidityChecker(si, obstacles)
    ss.setStateValidityChecker(validity_checker)

    # Set our robot's starting state
    start = ob.State(space)
    start[0] = start_pos[0]
    start[1] = start_pos[1]

    # Set our robot's goal state
    goal = ob.State(space)
    goal[0] = goal_pos[0]
    goal[1] = goal_pos[1]

    # Set the start and goal states
    ss.setStartAndGoalStates(start, goal)

    # Create the optimization objective (helper function is simply a switch statement) and set wind direction
    objective = ph.allocate_objective(si, objective_type)
    for i in range(objective.getObjectiveCount()):
        if type(objective.getObjective(i)) is ph.WindObjective:
            objective.getObjective(i).windDirectionDegrees = wind_direction_degrees
    ss.setOptimizationObjective(objective)

    # Construct the optimal planner (helper function is simply a switch statement)
    optimizing_planner = allocatePlanner(si, planner_type)
    ss.setPlanner(optimizing_planner)

    # Attempt to solve the planning problem in the given runtime
    solved = ss.solve(run_time)

    # Return the SimpleSetup object, which contains the solutionPath and spaceInformation
    # Must return ss, or else the object will be removed from memory
    return ss
