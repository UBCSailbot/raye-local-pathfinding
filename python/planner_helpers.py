import math
import sys
import numpy as np

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

# Balanced objective constants
LENGTH_WEIGHT = 100.0
CLEARANCE_WEIGHT = 1000.0
MIN_TURN_WEIGHT = 1.0
WIND_WEIGHT = 1.0

# Minimum turn cost multipliers
LARGE_TURN_MULTIPLIER = 500.0
SMALL_TURN_MULTIPLIER = 10.0

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0

class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, obstacles):
        super(ValidityChecker, self).__init__(si)
        self.obstacles = obstacles
        self.si = si

    # Returns whether the given state's position overlaps the ellipse
    def isValid(self, state):
        xy = [state.getX(), state.getY()]
        for obstacle in self.obstacles:
            if not obstacle.isValid(xy):
                return False
        return True

    # Returns the distance from the given state's position to the
    # boundary of the circular obstacle.
    def clearance(self, state):
        clearance = self.si.getStateSpace().getMaximumExtent()
        xy = [state.getX(), state.getY()]
        for obstacle in self.obstacles:
            if not obstacle.isValid(xy):
                return 0
            clearance = min(clearance, obstacle.clearance(xy))
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
    # by adding a large cost for large turns
    def motionCost(self, s1, s2):
        direction_radians = math.atan2(s2.getY() - s1.getY(), s2.getX() - s1.getX())

        # Calculate turn size
        try:
            turn_size_radians = abs_angle_dist_radians(direction_radians, self.last_direction_radians)
        except AttributeError:
            # Handle edge case first angle
            self.last_direction_radians = direction_radians
            turn_size_radians = abs_angle_dist_radians(direction_radians, self.last_direction_radians)

        # Calculate cost based on size of turn. Large turn is related to tacking angles.
        large_turn_threshold = math.radians(2 * max(UPWIND_MAX_ANGLE_DEGREES, DOWNWIND_MAX_ANGLE_DEGREES))
        if turn_size_radians > large_turn_threshold:
            return LARGE_TURN_MULTIPLIER * turn_size_radians
        else:
            return SMALL_TURN_MULTIPLIER * turn_size_radians


class WindObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(WindObjective, self).__init__(si, True)
        self.si_ = si
        self.windDirectionDegrees = 90 # north wind default

    # This objective function punishes the boat for going up/downwind
    def motionCost(self, s1, s2):
        distance = ((s2.getY() - s1.getY())**2 + (s2.getX() - s1.getX())**2)**0.5
        boatDirectionRadians = math.atan2(s2.getY() - s1.getY(), s2.getX() - s1.getX())

        if isUpwind(math.radians(self.windDirectionDegrees), boatDirectionRadians):
            return UPWIND_MULTIPLIER * distance
        elif isDownwind(math.radians(self.windDirectionDegrees), boatDirectionRadians):
            return DOWNWIND_MULTIPLIER * distance
        else:
            return 0.0

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
    opt.addObjective(minTurnObj, MIN_TURN_WEIGHT)
    opt.addObjective(windObj, WIND_WEIGHT)

    # REMOVING TO SAVE COMPUTATION AND SEE IF IMPROVES.
    opt.addObjective(lengthObj, LENGTH_WEIGHT)
    # opt.addObjective(clearObj, CLEARANCE_WEIGHT)

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

