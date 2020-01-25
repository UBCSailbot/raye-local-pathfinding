import math
import sys
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from math import sqrt

import planner_helpers as ph

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


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

def hasNoCollisions(localPath, obstacles, dimensions, myVector):

    print("Start of hasNoCollisions")
    space = ob.SE2StateSpace()

    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, dimensions[0])
    bounds.setLow(1, dimensions[1])

    bounds.setHigh(0, dimensions[2])
    bounds.setHigh(1, dimensions[3])

    # Set the bounds of space to be in [0,1].
    space.setBounds(bounds)

    # define a simple setup class
    ss = og.SimpleSetup(space)

    # Construct a space information instance for this state space
    si = ss.getSpaceInformation()

    # Set resolution of state validity checking. This is fraction of space's extent.
    # si.setStateValidityCheckingResolution(0.001)

    print("Middle")
    # Set the object used to check which states in the space are valid
    validity_checker = ph.ValidityChecker(si, obstacles)
    ss.setStateValidityChecker(validity_checker)

    print("End 0")
    print("myVector type: {}".format(type(myVector)))
    checkMotion = ss.getSpaceInformation().checkMotion(myVector, len(localPath))
    print("End 1")
    # states = []
    # for waypoint in localPath:
    #     states.append(createState(waypoint, space))

    # checkMotion = ss.getSpaceInformation().checkMotion(states, len(localPath))
    return checkMotion

def createState(waypoint, space):
    state = ob.State(space)
    state[0] = waypoint[0]
    state[1] = waypoint[1]
    return state

def plan(run_time, planner_type, objective_type, wind_direction, dimensions, start_pos, goal_pos, obstacles):
    # Construct the robot state space in which we're planning. We're
    # planning in [0,1]x[0,1], a subset of R^2.

    space = ob.SE2StateSpace()

    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, dimensions[0])
    bounds.setLow(1, dimensions[1])

    bounds.setHigh(0, dimensions[2])
    bounds.setHigh(1, dimensions[3])

    # Set the bounds of space to be in [0,1].
    space.setBounds(bounds)

    # define a simple setup class
    ss = og.SimpleSetup(space)

    # Construct a space information instance for this state space
    si = ss.getSpaceInformation()
# Set resolution of state validity checking. This is fraction of space's extent.
    # si.setStateValidityCheckingResolution(0.001)

    # Set the object used to check which states in the space are valid
    validity_checker = ph.ValidityChecker(si, obstacles)
    ss.setStateValidityChecker(validity_checker)

    # Set our robot's starting state to be the bottom-left corner of
    # the environment, or (0,0).
    start = ob.State(space)
    start[0] = start_pos[0]
    start[1] = start_pos[1]

    # Set our robot's goal state to be the top-right corner of the
    # environment, or (1,1).
    goal = ob.State(space)
    goal[0] = goal_pos[0]
    goal[1] = goal_pos[1]

    # Set the start and goal states
    ss.setStartAndGoalStates(start, goal)

    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    objective = ph.allocate_objective(si, objective_type)
    lengthObj = ob.PathLengthOptimizationObjective(si)
    clearObj = ph.ClearanceObjective(si)
    minTurnObj = ph.MinTurningObjective(si)
    windObj = ph.WindObjective(si)
    ss.setOptimizationObjective(objective)

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizing_planner = allocatePlanner(si, planner_type)

    ss.setPlanner(optimizing_planner)

    # Attempt to solve the planning problem in the given runtime
    final_cost = sys.maxsize
    ss.setPlanner(optimizing_planner)
    solved = ss.solve(run_time)
    final_cost = ss.getSolutionPath().cost(objective).value()
    print("Final cost = {}".format(final_cost))

    # Create solution list
    solution = []
    print("type(ss.getSolutionPath().getStates()): {}".format(type(ss.getSolutionPath().getStates())))
    print("print(ss.getSolutionPath().getStates()): {}".format(ss.getSolutionPath().getStates()))
    print("About to start loop 1")
    for state in ss.getSolutionPath().getStates():
        print("In loop")
        print("{}, {}, {}".format(state.getX(), state.getY(), state.getYaw()))
        solution.append((state.getX(), state.getY(), state.getYaw()))
    print("Done loop 1")

    # return (final_cost, ss.getSolutionPath().printAsMatrix(), solution, ss.getSolutionPath().getStates())
    # return s.getSolutionPath().getStates()
    return ss
