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

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
    def __str__(self):
        return str((self.x, self.y, self.radius))


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


def hasNoCollisions(localPathSS, obstacles):
    # Set the object used to check which states in the space are valid
    validity_checker = ph.ValidityChecker(localPathSS.getSpaceInformation(), obstacles)
    localPathSS.setStateValidityChecker(validity_checker)

    print("StateValidityCheckingResolution = {}".format(localPathSS.getSpaceInformation().getStateValidityCheckingResolution()))
    print("StateValidityCheckerObstaclesLength = {}".format(len(localPathSS.getStateValidityChecker().obstacles)))
    localPathSS.setup()
    print("SETUP")
    print("StateValidityCheckingResolution = {}".format(localPathSS.getSpaceInformation().getStateValidityCheckingResolution()))
    print("StateValidityCheckerObstaclesLength = {}".format(len(localPathSS.getStateValidityChecker().obstacles)))
    checkMotion = localPathSS.getSpaceInformation().checkMotion(localPathSS.getSolutionPath().getStates(), len(localPathSS.getSolutionPath().getStates()))
    print("Check motion is: {}".format(checkMotion))
    # motionValidator = localPathSS.getSpaceInformation().getMotionValidator()
    newCheckMotion = True
    stateSpace = localPathSS.getStateSpace()
    for stateIndex in range(len(localPathSS.getSolutionPath().getStates())):
        if stateIndex > 0:
            prevState = localPathSS.getSolutionPath().getState(stateIndex - 1)
            nextState = localPathSS.getSolutionPath().getState(stateIndex)
            
            # Check in between these points
            middleState = localPathSS.getSpaceInformation().allocState()
            resolution = localPathSS.getSpaceInformation().getStateValidityCheckingResolution()
            numPoints = 1 / resolution
            print("numPoints = {}".format(numPoints))
            for i in range(int(numPoints + 1)):
                fraction = i * resolution
                stateSpace.interpolate(prevState, nextState, fraction, middleState)
                newCheckMotion = newCheckMotion and localPathSS.getSpaceInformation().isValid(middleState)
                print(newCheckMotion)


    ax = plt.gca()
    for obstacle in obstacles:
        print("Obstacle {}, {}, {}".format(obstacle.x, obstacle.y, obstacle.radius))
        c = plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius)
        c.set_color('r')
        ax.add_patch(c)
    for s in localPathSS.getSolutionPath().getStates():
        print("State {}, {}".format(s.getX(), s.getY()))
        c = plt.Circle((s.getX(), s.getY()), radius=0.1)
        c.set_color('g')
        ax.add_patch(c)

    plt.show()
    # return checkMotion
    return newCheckMotion


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
    # si.setStateValidityCheckingResolution(0.001)

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
