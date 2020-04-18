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

VALIDITY_CHECKING_RESOLUTION = 0.001  # Default 0.01

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


# Method that looks numLookAheadWaypoints ahead on path, starting from positionXY and nextLocalWaypointIndex.
# Returns the index of the waypoint in which there is an obstacle when going to it.
# Returns -1 if there are no obstacles on path.
def indexOfObstacleOnPath(positionXY, nextLocalWaypointIndex, numLookAheadWaypoints, omplPath):
    # Setup for obstacle-on-path checking
    stateSpace = omplPath.getStateSpace()
    solutionPath = omplPath.getSolutionPath()
    spaceInformation = omplPath.getSpaceInformation()

    # Handle strange cases with less than 2 states
    if len(solutionPath.getStates()) <= 1:
        print("WARNING: len(solutionPath.getStates()) = {}. Expected >1.".format(len(solutionPath.getStates())))
        if len(solutionPath.getStates()) == 0:
            return -1
        else:
            hasObstacle = (not spaceInformation.isValid(solutionPath.getState(0)))
            return 0 if hasObstacle else -1

    # Check if initial state is valid, as checkMotion can't check that
    firstState = spaceInformation.allocState()
    firstState.setXY(positionXY[0], positionXY[1])
    if not spaceInformation.isValid(firstState):
        return -1

    # Get the relevant states (ignore past states and use current position as first state)
    relevantStates = []
    relevantStates.append(firstState)
    for i in range(numLookAheadWaypoints):
        stateIndex = nextLocalWaypointIndex + i
        relevantStates.append(solutionPath.getState(stateIndex))

    # Interpolate between states and check for validity
    for stateIndex in range(1, len(relevantStates)):
        # Check in between these points
        prevState = relevantStates[stateIndex - 1]
        nextState = relevantStates[stateIndex]
        hasObstacle = (not spaceInformation.checkMotion(prevState, nextState))
        if hasObstacle:
            return stateIndex


    '''Uncomment to visualize the obstacles, relevant states, and all states
    ax = plt.gca()
    for obstacle in obstacles:
        # print("Obstacle {}, {}, {}".format(obstacle.x, obstacle.y, obstacle.radius))
        c = plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius)
        c.set_color('r')
        ax.add_patch(c)
    for s in omplPath.getSolutionPath().getStates():
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

    return -1

def plan(run_time, planner_type, objective_type, wind_direction_degrees, dimensions, start_pos, goal_pos, obstacles):
    # Construct the robot state space in which we're planning
    space = ob.SE2StateSpace()

    # Create bounds on the position
    bounds = ob.RealVectorBounds(2)
    x_min, y_min, x_max, y_max = dimensions
    bounds.setLow(0, x_min)
    bounds.setLow(1, y_min)
    bounds.setHigh(0, x_max)
    bounds.setHigh(1, y_max)
    space.setBounds(bounds)

    # Define a simple setup class
    ss = og.SimpleSetup(space)

    # Construct a space information instance for this state space
    si = ss.getSpaceInformation()

    # Set resolution of state validity checking, which is fraction of space's extent (Default is 0.01)
    si.setStateValidityCheckingResolution(VALIDITY_CHECKING_RESOLUTION)

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
        if isinstance(objective.getObjective(i), ph.WindObjective):
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
