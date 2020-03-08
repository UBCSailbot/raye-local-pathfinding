import math
import sys

from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from math import sqrt

##
## planner_helpers.py
## ------------------
## This file contains helper functions
## for the OMPL planner, such as the 
## ValidityChecker and the objective
## functions.
##

# Upwind downwind constants
UPWIND_MAX_ANGLE_DEGREES = 30.0
DOWNWIND_MAX_ANGLE_DEGREES = 30.0

class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, obstacles):
        super(ValidityChecker, self).__init__(si)
        self.obstacles = obstacles

    # Returns whether the given state's position overlaps the
    # circular obstacle
#    def isValid(self, state):
#        x = state.getX()
#        y = state.getY()
#        for obstacle in self.obstacles:
#            if sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2)) - obstacle.radius <= 0:
#                return False
#
#        return True

    def isValid(self, state):
        M  = np.array([state.getX(), state.getY()])
        for obstacle in self.obstacles:
            A = np.array(A)
            B = np.array(B)
            C = np.array(C)
            AB = B - A 
            AM = M - A
            BC = C- B
            BM = M - B
            if not (0 <= AB.dot(AM) and AB.dot(AM) <= AB.dot(AB)) and (0 <= BC.dot(BM) and BC.dot(BM) <= BC.dot(BC)):
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
#        for obstacle in self.obstacles:
#
#            clearance += (sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2)) - obstacle.radius)
#            if clearance <= 0:
#                return 0
#
        return clearance


class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        if (self.si_.getStateValidityChecker().clearance(s) == 0):
            return sys.maxsize
        return ob.Cost(1 / (self.si_.getStateValidityChecker().clearance(s))**0.5)


class MinTurningObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(MinTurningObjective, self).__init__(si, True)
        self.si_ = si

    # This objective function punishes the boat for tacking or jibing
    # by adding a 500 cost for large turns
    def motionCost(self, s1, s2):
        direction_radians = math.atan2(s2.getY() - s1.getY(), s2.getX() - s1.getX())
        try:
            diff_radians = abs_angle_dist_radians(direction_radians, self.last_direction_radians)
        except AttributeError:
            self.last_direction_radians = direction_radians
            diff_radians = abs_angle_dist_radians(direction_radians, self.last_direction_radians)
        if diff_radians > math.radians(2 * max(UPWIND_MAX_ANGLE_DEGREES, DOWNWIND_MAX_ANGLE_DEGREES)):
            return 500.0
        else:
            return diff_radians*10


class WindObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(WindObjective, self).__init__(si, True)
        self.si_ = si
        self.windDirectionDegrees = 90 # north wind default

    # This objective function punishes the boat for going up/downwind
    def motionCost(self, s1, s2):
        distance = ((s2.getY() - s1.getY())**2 + (s2.getX() - s1.getX())**2)**0.5
        boatDirectionRadians = math.atan2(s2.getY() - s1.getY(), s2.getX() - s1.getX())

        isUpwindOrDownwind = isUpwind(math.radians(self.windDirectionDegrees), boatDirectionRadians) or isDownwind(math.radians(self.windDirectionDegrees), boatDirectionRadians)
        return sys.maxsize * distance if isUpwindOrDownwind else 0.0


def isUpwind(windDirectionRadians, boatDirectionRadians):
    diffRadians = abs_angle_dist_radians(windDirectionRadians, boatDirectionRadians)
    return math.fabs(diffRadians - math.radians(180)) < math.radians(UPWIND_MAX_ANGLE_DEGREES)

def isDownwind(windDirectionRadians, boatDirectionRadians):
    diffRadians = abs_angle_dist_radians(windDirectionRadians, boatDirectionRadians)
    return math.fabs(diffRadians) < math.radians(DOWNWIND_MAX_ANGLE_DEGREES)

def abs_angle_dist_radians(angle1_radians, angle2_radians):
    # Absolute distance between angles
    fabs = math.fabs(math.atan2(math.sin(angle1_radians - angle2_radians), math.cos(angle1_radians - angle2_radians)))
    return fabs


def get_clearance_objective(si):
    return ClearanceObjective(si)

def get_path_length_objective(si):
    return ob.PathLengthOptimizationObjective(si)

def get_threshold_path_length_objective(si):
    obj = ob.PathLengthOptimizationObjective(si)
    # obj.setCostThreshold(ob.Cost(8))
    return obj

def getBalancedObjective(si):
    lengthObj = ob.PathLengthOptimizationObjective(si)
    clearObj = ClearanceObjective(si)
    minTurnObj = MinTurningObjective(si)
    windObj = WindObjective(si)

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, 1.0)
    opt.addObjective(clearObj, 1.0)
    opt.addObjective(minTurnObj, 1.0)
    opt.addObjective(windObj, 1.0)
    # opt.setCostThreshold(ob.Cost(5))

    return opt

# Keep these in alphabetical order and all lower case
def allocate_objective(si, objectiveType):
    if objectiveType.lower() == "pathclearance":
        return get_clearance_objective(si)
    elif objectiveType.lower() == "pathlength":
        return get_path_length_objective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return get_threshold_path_length_objective(si)
    elif objectiveType.lower() == "weightedlengthandclearancecombo":
        return getBalancedObjective(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")

