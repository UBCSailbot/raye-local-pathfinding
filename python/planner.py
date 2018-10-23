import math

from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
import argparse

import numpy as np
import matplotlib.pyplot as plt

BOUNDS = 1

CIRCLE_RADIUS = .15


class ValidityChecker(ob.StateValidityChecker):
    # Returns whether the given state's position overlaps the
    # circular obstacle
    def isValid(self, state):
        return self.clearance(state) > 0.0

    # Returns the distance from the given state's position to the
    # boundary of the circular obstacle.
    def clearance(self, state):
        # Extract the robot's (x,y) position from its state
        x = state.getX()
        y = state.getY()

        # Distance formula between two points, offset by the circle's
        # radius
        pos = 0.5
        size = CIRCLE_RADIUS
        return sqrt((x - pos) * (x - pos) + (y - pos) * (y - pos)) - size


class WindObjective(ob.PathLengthOptimizationObjective):
    def __init__(self, si, windDirection=math.pi / 4):
        super(WindObjective, self).__init__(si)
        self.si_ = si
        self.windDirection = windDirection

    def motionCost(self, state1, state2):
        angleBetweenPoints = math.atan2(state2.getY() - state1.getY(), state2.getX() - state1.getX())
        angles = absDistanceBetweenAngles(self.windDirection, angleBetweenPoints)

        windCost = pow(1 - angles / math.pi, 3)

        print(state1.getX(), ",", state1.getY(), " and ", state2.getX(), ",", state2.getY(), " with angle ",
              math.degrees(angleBetweenPoints), "and Distance ", math.degrees(angles), " make ", windCost)
        return ob.Cost(windCost * super(WindObjective, self).motionCost(state1, state2).value())


class AngleMotionValidator(ob.DiscreteMotionValidator):
    def __init__(self, si):
        super(AngleMotionValidator, self).__init__(si)
        self.si_ = si

    def checkMotion(self, state1, state2, lastValid):
        return self.isValidAngle(state1, state2) and super(AngleMotionValidator, self).checkMotion(state1, state2,
                                                                                                   lastValid)

    def checkMotion(self, state1, state2):
        return self.isValidAngle(state1, state2) and super(AngleMotionValidator, self).checkMotion(state1, state2)

    def isValidAngle(self, state1, state2):
        angleBetweenPoints = math.atan2(state2.getY() - state1.getY(), state2.getX() - state1.getX())
        differenceBetweenAnglePathAndStartingAngle = absDistanceBetweenAngles(angleBetweenPoints, state1.getYaw())
        return differenceBetweenAnglePathAndStartingAngle < 0.1


def absDistanceBetweenAngles(angle1, angle2):
    a = angle1 - angle2

    if a > math.pi:
        a -= math.pi * 2
    if a < -math.pi:
        a += math.pi * 2
    fabs = math.fabs(a)
    return fabs


def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)


def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj


class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # but we want to represent the objective as a path cost
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        return ob.Cost(1 / self.si_.getStateValidityChecker().clearance(s))


def getSailbotObjective(si):
    windObj = WindObjective(si)
    distanceObj = ob.PathLengthOptimizationObjective(si)

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(windObj, 1.0)
    opt.addObjective(distanceObj, 1.0)

    return opt


def getClearanceObjective(si):
    return ClearanceObjective(si)


def getPathLengthObjWithCostToGo(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj


# Keep these in alphabetical order and all lower case
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


# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
    if objectiveType.lower() == "sailbot":
        return getSailbotObjective(si)
    if objectiveType.lower() == "pathclearance":
        return getClearanceObjective(si)
    elif objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    elif objectiveType.lower() == "weightedlengthandclearancecombo":
        return getBalancedObjective1(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")


def createNumpyPath(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))

    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array


def plan(runTime, plannerType, objectiveType):
    # Construct the robot state space in which we're planning. We're
    # planning in [0,1]x[0,1], a subset of R^2.
    space = ob.SE2StateSpace()

    bounds = ob.RealVectorBounds(2)
    bounds.setHigh(0, BOUNDS)
    bounds.setHigh(1, BOUNDS)

    bounds.setLow(0, 0)
    bounds.setLow(1, 0)

    # Set the bounds of space to be in [0,1].
    space.setBounds(bounds)
    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    si.setStateValidityCheckingResolution(0.0001)
    validator = AngleMotionValidator(si)
    si.setMotionValidator(validator)

    # Set the object used to check which states in the space are valid
    validityChecker = ValidityChecker(si)
    si.setStateValidityChecker(validityChecker)

    si.setup()

    # Set our robot's starting state to be the bottom-left corner of
    # the environment, or (0,0).
    start = ob.State(space)
    start[0] = 0.0
    start[1] = 0.0
    start[2] = math.pi / 4

    # Set our robot's goal state to be the top-right corner of the
    # environment, or (1,1).
    goal = ob.State(space)
    goal[0] = BOUNDS
    goal[1] = BOUNDS
    goal[2] = math.pi / 4

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)

    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizingPlanner = allocatePlanner(si, plannerType)

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(runTime)

    if solved:
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization '
              'objective value of {2:.4f}'.format(
            optimizingPlanner.getName(),
            pdef.getSolutionPath().length(),
            pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()))
        print(pdef.getSolutionPath().printAsMatrix())
        plotPath(pdef)

    else:
        print("No solution found.")


def plotPath(pdef):
    matrix = pdef.getSolutionPath().printAsMatrix()
    path = createNumpyPath(matrix)
    x, y = path.T
    ax = plt.gca()
    ax.plot(x, y)
    ax.axis(ymax=BOUNDS, xmax=BOUNDS)
    ax.add_patch(plt.Circle((.5, .5), radius=CIRCLE_RADIUS))
    plt.show()


if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Sailbot motion planning CLI.')

    # Add a filename argument
    parser.add_argument('-t', '--runtime', type=float, default=1.0, help=
    '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
    parser.add_argument('-p', '--planner', default='RRTstar',
                        choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar',
                                 'SORRTstar'],
                        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    parser.add_argument('-o', '--objective', default='Sailbot',
                        choices=['PathClearance', 'PathLength', 'ThresholdPathLength',
                                 'WeightedLengthAndClearanceCombo', 'Sailbot'],
                        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')
    parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2],
                        help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
                             ' Defaults to WARN.')

    # Parse the arguments
    args = parser.parse_args()

    # Check that time is positive
    if args.runtime <= 0:
        raise argparse.ArgumentTypeError(
            "argument -t/--runtime: invalid choice: %r (choose a positive number greater than 0)" \
            % (args.runtime,))

    # Set the log level
    if args.info == 0:
        ou.setLogLevel(ou.LOG_WARN)
    elif args.info == 1:
        ou.setLogLevel(ou.LOG_INFO)
    elif args.info == 2:
        ou.setLogLevel(ou.LOG_DEBUG)
    else:
        ou.OMPL_ERROR("Invalid log-level integer.")

    # Solve the planning problem
    plan(args.runtime, args.planner, args.objective)
