import math

from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
import argparse
import sys

import numpy as np
import matplotlib.pyplot as plt


class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


def parse_obstacle(s):
    try:
        x, y, radius = map(float, s.split(','))
        return Obstacle(x, y, radius)
    except Exception:
        raise argparse.ArgumentTypeError("Obstacles must be x,y,radius")


class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, obstacles):
        super(ValidityChecker, self).__init__(si)
        self.obstacles = obstacles

    # Returns whether the given state's position overlaps the
    # circular obstacle
    def isValid(self, state):
        x = state.getX()
        y = state.getY()
        for obstacle in self.obstacles:
            if sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2)) - obstacle.radius <= 0:
                return False

        return True

    # Returns the distance from the given state's position to the
    # boundary of the circular obstacle.
    def clearance(self, state):
        if len(self.obstacles) == 0:
            return 1

        # Extract the robot's (x,y) position from its state
        x = state.getX()
        y = state.getY()

        clearance = 0
        # Distance formula between two points, offset by the circle's
        # radius
        for obstacle in self.obstacles:

            clearance += sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2)) - obstacle.radius
            if clearance <= 0:
                return 0

        return clearance


class WindObjective(ob.PathLengthOptimizationObjective):
    def __init__(self, si, wind_direction=math.pi / 2):
        super(WindObjective, self).__init__(si)
        self.si_ = si
        self.windDirection = wind_direction

    def motionCost(self, state1, state2):
        angle_between_points = math.atan2(state2.getY() - state1.getY(), state2.getX() - state1.getX())
        angles = absolute_distance_between_angles(self.reverseAngle(self.windDirection), angle_between_points)

        wind_cost = pow(1 - angles / math.pi, 2)
        return ob.Cost(wind_cost * super(WindObjective, self).motionCost(state1, state2).value())

    def reverseAngle(self, angle):
        if angle <= 0:
            return angle + math.pi
        else:
            return angle - math.pi


class AngleMotionValidator(ob.DiscreteMotionValidator):
    def __init__(self, si):
        super(AngleMotionValidator, self).__init__(si)
        self.si_ = si

    def checkMotion(self, state1, state2, lastValid):
        return self.is_valid_angle(state1, state2) and super(AngleMotionValidator, self).checkMotion(state1, state2,
                                                                                                     lastValid)

    def checkMotion(self, state1, state2):
        return self.is_valid_angle(state1, state2) and super(AngleMotionValidator, self).checkMotion(state1, state2)

    def is_valid_angle(self, state1, state2):
        angle_between_points = math.atan2(state2.getY() - state1.getY(), state2.getX() - state1.getX())
        difference_between_angle_path_and_starting_angle = absolute_distance_between_angles(angle_between_points,
                                                                                            state1.getYaw())
        return difference_between_angle_path_and_starting_angle < 0.05


def absolute_distance_between_angles(angle1, angle2):
    fabs = math.fabs(math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2)))
    return fabs


def get_path_length_objective(si):
    return ob.PathLengthOptimizationObjective(si)


def get_threshold_path_length_objective(si):
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
        if(self.si_.getStateValidityChecker().clearance(s)==0):
            return sys.maxsize
        return ob.Cost(1 / self.si_.getStateValidityChecker().clearance(s))


def getSailbotObjective(si, wind_direction):
    wind_obj = WindObjective(si, wind_direction)
    distance_obj = ob.PathLengthOptimizationObjective(si)
    clearance_obj = get_clearance_objective(si);

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(wind_obj, 1.0)
    opt.addObjective(distance_obj, 1.0)
    opt.addObjective(clearance_obj, 0.1)

    return opt


def get_clearance_objective(si):
    return ClearanceObjective(si)


def get_path_length_obj_with_cost_to_go(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj


# Keep these in alphabetical order and all lower case
def allocate_planner(si, planner_type):
    if planner_type.lower() == "bfmtstar":
        return og.BFMT(si)
    elif planner_type.lower() == "bitstar":
        return og.BITstar(si)
    elif planner_type.lower() == "fmtstar":
        return og.FMT(si)
    elif planner_type.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif planner_type.lower() == "prmstar":
        return og.PRMstar(si)
    elif planner_type.lower() == "rrtstar":
        return og.RRTstar(si)
    elif planner_type.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# Keep these in alphabetical order and all lower case
def allocate_objective(si, objectiveType, windDirection):
    if objectiveType.lower() == "sailbot":
        return getSailbotObjective(si, windDirection)
    if objectiveType.lower() == "pathclearance":
        return get_clearance_objective(si)
    elif objectiveType.lower() == "pathlength":
        return get_path_length_objective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return get_threshold_path_length_objective(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")


def create_numpy_path(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))

    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array


def plan(run_time, planner_type, objective_type, wind_direction, dimensions, obstacles):
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
    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    si.setStateValidityCheckingResolution(0.0001)
    validator = AngleMotionValidator(si)
    si.setMotionValidator(validator)

    # Set the object used to check which states in the space are valid
    validity_checker = ValidityChecker(si, obstacles)
    si.setStateValidityChecker(validity_checker)

    si.setup()

    # Set our robot's starting state to be the bottom-left corner of
    # the environment, or (0,0).
    start = ob.State(space)
    start[0] = dimensions[0]
    start[1] = dimensions[1]
    start[2] = math.pi / 4

    # Set our robot's goal state to be the top-right corner of the
    # environment, or (1,1).
    goal = ob.State(space)
    goal[0] = dimensions[2]
    goal[1] = dimensions[3]
    goal[2] = math.pi / 4

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal, 0.1)

    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocate_objective(si, objective_type, wind_direction))

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizing_planner = allocate_planner(si, planner_type)

    # Set the problem instance for our planner to solve
    optimizing_planner.setProblemDefinition(pdef)
    optimizing_planner.setup()

    # attempt to solve the planning problem in the given runtime
    solved = optimizing_planner.solve(run_time)

    if solved:
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization '
              'objective value of {2:.4f}'.format(optimizing_planner.getName(),
                                                  pdef.getSolutionPath().length(),
                                                  pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()))
        print(pdef.getSolutionPath().printAsMatrix())
        plot_path(pdef, dimensions, obstacles)

    else:
        print("No solution found.")


def plot_path(pdef, dimensions, obstacles):
    matrix = pdef.getSolutionPath().printAsMatrix()
    path = create_numpy_path(matrix)
    x, y = path.T
    ax = plt.gca()
    ax.plot(x, y)
    ax.axis(xmin=dimensions[0], xmax=dimensions[2], ymin=dimensions[1], ymax=dimensions[3])
    for obstacle in obstacles:
        ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))

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
    parser.add_argument('-w', '--windDirection', type=lambda x: math.degrees(int(x)), default=-135,
                        help='(Optional) Wind direction in degrees')
    parser.add_argument('-d', '--dimensions', nargs=4, type=int, default=[0, 0, 5, 5],
                        help='(Optional) dimensions of the space')
    parser.add_argument('-ob', '--obstacles', nargs='+', type=parse_obstacle, default=[parse_obstacle("2.5,2.5,1")],
                        help='(Optional) dimensions of the space')

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
    plan(args.runtime, args.planner, args.objective, args.windDirection, args.dimensions, args.obstacles)
