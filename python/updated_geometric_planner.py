import math
import sys

from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from math import sqrt

import planner_helpers as ph

from plotting import plot_path


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

def plan(run_time, planner_type, objective_type, wind_direction, dimensions, goal_pos, obstacles):
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
    start[0] = 0
    start[1] = 0

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
    print("ss.getProblemDefinition().hasOptimizationObjective( ){}".format(ss.getProblemDefinition().hasOptimizationObjective()))
    print("ss.getProblemDefinition().hasOptimizedSolution() {}".format(ss.getProblemDefinition().hasOptimizedSolution()))

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizing_planner = allocatePlanner(si, planner_type)

    ss.setPlanner(optimizing_planner)

    # attempt to solve the planning problem in the given runtime
    # (but if the solution is terrible, try again)
    final_cost = sys.maxsize
    while final_cost > 100000:
        ss.setPlanner(optimizing_planner)
        solved = ss.solve(run_time)
        final_cost = ss.getSolutionPath().cost(objective).value()

    if solved:
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization '
              'objective value of {2:.4f}'.format(ss.getPlanner().getName(),
                                                  ss.getSolutionPath().length(),
                                                  0.1))
        solution_path = ss.getSolutionPath()
        states = solution_path.getStates()
        prevState = states[0]
        for state in states[1:]:
            print(space.validSegmentCount(prevState, state))
            prevState = state


        print("ss.getSolutionPath().printAsMatrix() = {}".format(ss.getSolutionPath().printAsMatrix()))
        print("ss.haveSolutionPath() = {}".format(ss.haveSolutionPath()))
        print("ss.haveExactSolutionPath() = {}".format(ss.haveExactSolutionPath()))
        print("***")
        print("ss.getSolutionPath().length() = {}".format(ss.getSolutionPath().length()))
        print("ss.getSolutionPath().check() = {}".format(ss.getSolutionPath().check()))
        print("ss.getSolutionPath().clearance() = {}".format(ss.getSolutionPath().clearance()))
        print("ss.getSolutionPath().cost(objective).value() = {}".format(ss.getSolutionPath().cost(objective).value()))
        print("ss.getSolutionPath().cost(lengthObj).value() = {}".format(ss.getSolutionPath().cost(lengthObj).value()))
        print("ss.getSolutionPath().cost(clearObj).value() = {}".format(ss.getSolutionPath().cost(clearObj).value()))
        print("ss.getSolutionPath().cost(minTurnObj).value() = {}".format(ss.getSolutionPath().cost(minTurnObj).value()))
        print("ss.getSolutionPath().cost(windObj ).value() = {}".format(ss.getSolutionPath().cost(windObj).value()))
        print("ss.getProblemDefinition().hasOptimizedSolution() {}".format(ss.getProblemDefinition().hasOptimizedSolution()))
        plot_path(ss.getSolutionPath(), dimensions, obstacles)
        print("***")

    else:
        print("No solution found.")

