#!/usr/bin/env python
from datetime import datetime
from datetime import date
import os
import rospy
import time
import math
import numpy as np
import planner_helpers as plans
from geopy.distance import distance
from sailbot_msg.msg import latlon

# Location constants
PORT_RENFREW_LATLON = latlon(48.5, -124.8)
MAUI_LATLON = latlon(20.0, -156.0)

# Constants
PATH_UPDATE_TIME_LIMIT_SECONDS = 7200
AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM = 3.0
PI_DEG = 180.0
KNOTS_TO_KMPH = 1.852

# Scale NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES and NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND to change based on
# waypoint distance
# "As far as the eye can see" is ~3 miles = ~5 km
LOOK_AHEAD_FOR_OBSTACLES_KM = 5.0
NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES = int(math.ceil(LOOK_AHEAD_FOR_OBSTACLES_KM /
                                                       AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM))
LOOK_AHEAD_FOR_UPWIND_DOWNWIND_KM = 5.0
NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND = int(math.ceil(LOOK_AHEAD_FOR_UPWIND_DOWNWIND_KM /
                                                             AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM))

# Constants for bearing and heading
BEARING_NORTH = 0
BEARING_EAST = 90
BEARING_SOUTH = 180
BEARING_WEST = 270

HEADING_EAST = 0
HEADING_NORTH = 90
HEADING_WEST = 180
HEADING_SOUTH = 270

# Constants for boat frame angles
BOAT_RIGHT = 0
BOAT_FORWARD = 90
BOAT_LEFT = 180
BOAT_BACKWARD = 270

# Constants for pathfinding updates
COST_THRESHOLD_PER_KM = 650


def takeScreenshot(return_path_to_file=False):
    '''Take a screenshot and save it to a png file.

    Args:
       return_path_to_file (bool): If True, will return path to saved screenshot file

    Returns:
       str path to saved screenshot file if return_path_to_file is True, else nothing

    Note: The first time this is called, it will create the following folder structure if it does not already exist:
          images/(%b-%d-%Y)/(%H-%M-%S) where %b-%d-%Y is a date string and %H-%M-%S is a time string.

          Then it will save all image files into this folder with a filename that is time of the screenshot.
    '''
    # Put import here to avoid CI issues with pyautogui
    import pyautogui

    # Set imagePath on first time
    try:
        takeScreenshot.imagePath
    except AttributeError:
        rospy.loginfo("Handling first time case in takeScreenshot()")
        pathToThisFile = os.path.dirname(os.path.abspath(__file__))
        dateString = date.today().strftime("%b-%d-%Y")
        timeString = datetime.now().strftime('%H-%M-%S')
        pathToDateFolder = "{}/../images/{}".format(pathToThisFile, dateString)
        pathToStartTimeFolder = "{}/{}".format(pathToDateFolder, timeString)
        if not os.path.exists(pathToDateFolder):
            os.mkdir(pathToDateFolder)
        if not os.path.exists(pathToStartTimeFolder):
            os.mkdir(pathToStartTimeFolder)
        takeScreenshot.imagePath = pathToStartTimeFolder

    # Take screenshot
    time.sleep(1)
    myScreenshot = pyautogui.screenshot()

    # Save screenshot
    timeString = datetime.now().strftime('%H-%M-%S')
    fullImagePath = "{}/{}.png".format(takeScreenshot.imagePath, timeString)
    rospy.loginfo("Taking screenshot...")
    myScreenshot.save(fullImagePath)
    rospy.loginfo("Screenshot saved to {}".format(fullImagePath))
    if return_path_to_file:
        return fullImagePath


def absAngleDegreesBetweenLatlons(latlon1, latlon2, referenceLatlon):
    '''Calculates the absolute angle between the XY coordinates of two latlons, given referenceLatlon located at (0,0)

    Args:
        latlon1 (sailbot_msg.msg._latlon.latlon): The first latlon to calculate the degree of
        latlon2 (sailbot_msg.msg._latlon.latlon): The second latlon to calculate the degree of
        referenceLatlon (sailbot_msg.msg._latlon.latlon): The latlon that will be located at (0, 0)

    Returns:
        float representing the absolute angle between the two latlons in degrees
    '''
    x1, y1 = latlonToXY(latlon1, referenceLatlon)
    x2, y2 = latlonToXY(latlon2, referenceLatlon)
    return math.degrees(plans.abs_angle_dist_radians(math.atan2(y2, x2), math.atan2(y1, x1)))


def latlonToXY(latlon, referenceLatlon):
    '''Calculate the xy (km) coordinates of the given latlon, given the referenceLatlon located at (0,0)

    Args:
       latlon (sailbot_msg.msg._latlon.latlon): The latlon whose xy coordinates will be calculated.
       referenceLatlon (sailbot_msg.msg._latlon.latlon): The latlon that will be located at (0,0).

    Returns:
       list [x,y] representing the position of latlon in xy (km) coordinates
    '''
    x = distance((referenceLatlon.lat, referenceLatlon.lon), (referenceLatlon.lat, latlon.lon)).kilometers
    y = distance((referenceLatlon.lat, referenceLatlon.lon), (latlon.lat, referenceLatlon.lon)).kilometers
    if referenceLatlon.lon > latlon.lon:
        x = -x
    if referenceLatlon.lat > latlon.lat:
        y = -y
    return [x, y]


def XYToLatlon(xy, referenceLatlon):
    '''Calculate the latlon coordinates of the given xy, given the referenceLatlon located at (0,0)

    Args:
       xy (list of two floats): The xy (km) coordinates whose latlon coordinates will be calculated.
       referenceLatlon (sailbot_msg.msg._latlon.latlon): The latlon that will be located at (0,0) wrt xy.

    Returns:
       sailbot_msg.msg._latlon.latlon representing the position of xy in latlon coordinates
    '''
    x_distance = distance(kilometers=xy[0])
    y_distance = distance(kilometers=xy[1])
    destination = x_distance.destination(point=(referenceLatlon.lat, referenceLatlon.lon), bearing=BEARING_EAST)
    destination = y_distance.destination(point=(destination.latitude, destination.longitude), bearing=BEARING_NORTH)
    return latlon(destination.latitude, destination.longitude)


def timeLimitExceeded(lastTimePathCreated, speedup):
    '''Checks the time since last path was created has been too long. Ensures the boat is
    not stuck to a poor, stagnant path

    Args:
       lastTimePathCreated (float): Time that the last path was created
       speedup (float): Current speedup value

    Returns:
       bool True iff the time difference between now and lastTimePathCreated exceeds a time limit
    '''
    # TODO: Figure out a way to make changing speedup work properly for this
    # Shorter time limit when there is speedup
    pathUpdateTimeLimitSecondsSpeedup = PATH_UPDATE_TIME_LIMIT_SECONDS / speedup
    if time.time() - lastTimePathCreated > pathUpdateTimeLimitSecondsSpeedup:
        rospy.logwarn("{} seconds elapsed. Time limit of {} seconds was exceeded.".format(
            pathUpdateTimeLimitSecondsSpeedup, PATH_UPDATE_TIME_LIMIT_SECONDS))
        return True
    else:
        return False


def getDesiredHeadingDegrees(position, localWaypoint):
    '''Calculate the heading that the boat should aim towards to reach the local waypoint.

    Args:
       position (sailbot_msg.msg._latlon.latlon): Current position of the boat
       localWaypoint (sailbot_msg.msg._latlon.latlon): Current local waypoint that the boat is aiming towards

    Returns:
       float that is the heading (degrees) that the boat should be aiming towards to reach the local waypoint
    '''
    xy = latlonToXY(localWaypoint, position)
    return math.degrees(math.atan2(xy[1], xy[0]))


def getLOSHeadingDegrees(position, previousWaypoint, currentWaypoint):
    '''Calculate the heading that the boat should aim towards to reach the local waypoint.
    The LOS guidance law will attempt to keep the boat on the straight line between the previous and current waypoint.

    Args:
       position (sailbot_msg.msg._latlon.latlon): Current position of the boat
       previousWaypoint (sailbot_msg.msg._latlon.latlon): Previous local waypoint that the boat visited
       currentWaypoint (sailbot_msg.msg._latlon.latlon): Current local waypoint that the boat is aiming towards

    Returns:
       float that is the heading (degrees) that the boat should be aiming towards to reach the local waypoint
    '''
    # The lookahead distance describes how "hard" the boat should try to reduce the crosstrack error
    lookahead = 0.001
    Kp = 1 / lookahead

    x_start = previousWaypoint.lon
    y_start = previousWaypoint.lat

    x_end = currentWaypoint.lon
    y_end = currentWaypoint.lat

    x = position.lon
    y = position.lat

    # The direction that we would ideally be heading
    straight_course = math.atan2(y_end - y_start, x_end - x_start)

    # Lateral error, i. e. how far the boat has drifted off track
    crosstrack_error = -(x - x_start) * math.sin(straight_course) + (y - y_start) * math.cos(straight_course)

    return math.degrees(straight_course - math.atan(Kp * crosstrack_error))


def measuredWindToGlobalWind(measuredWindSpeed, measuredWindDirectionDegrees, boatSpeed, headingDegrees):
    '''Calculate the global wind based on the measured wind and the boat velocity
    Args:
       measuredWindSpeed (float): speed of the wind measured from the boat. All speed values must be in the same units.
       measuredWindDirectionDegrees (float): angle of the measured with wrt the boat.
                                             0 degrees is wind blowing to the right. 90 degrees is wind blowing forward.
       boatSpeed (float): speed of the boat
       headingDegrees (float): angle of the boat in global frame. 0 degrees is East. 90 degrees is North.
    Returns:
       float, float pair representing the globalWindSpeed (same units as input speed), globalWindDirectionDegrees
    '''
    measuredWindRadians, headingRadians = math.radians(measuredWindDirectionDegrees), math.radians(headingDegrees)

    # GF = global frame. BF = boat frame
    # Calculate wind speed in boat frame. X is right. Y is forward.
    measuredWindSpeedXBF = measuredWindSpeed * math.cos(measuredWindRadians)
    measuredWindSpeedYBF = measuredWindSpeed * math.sin(measuredWindRadians)

    # Assume boat is moving entirely in heading direction, so all boat speed is in boat frame Y direction
    trueWindSpeedXBF = measuredWindSpeedXBF
    trueWindSpeedYBF = measuredWindSpeedYBF + boatSpeed

    # Calculate wind speed in global frame. X is EAST. Y is NORTH.
    trueWindSpeedXGF = trueWindSpeedXBF * math.sin(headingRadians) + trueWindSpeedYBF * math.cos(headingRadians)
    trueWindSpeedYGF = trueWindSpeedYBF * math.sin(headingRadians) - trueWindSpeedXBF * math.cos(headingRadians)

    # Calculate global wind speed and direction
    globalWindSpeed = (trueWindSpeedXGF**2 + trueWindSpeedYGF**2)**0.5
    globalWindDirectionDegrees = math.degrees(math.atan2(trueWindSpeedYGF, trueWindSpeedXGF))

    return globalWindSpeed, globalWindDirectionDegrees


def globalWindToMeasuredWind(globalWindSpeed, globalWindDirectionDegrees, boatSpeed, headingDegrees):
    '''Calculate the measured wind based on the global wind and the boat velocity

    Args:
       globalWindSpeed (float): speed of the global wind. All speed values should be in the same units.
       globalWindDirectionDegrees (float): angle of the global wind in the global frame.
                                           0 degrees is East. 90 degrees is North.
       boatSpeed (float): speed of the boat
       headingDegrees (float): angle of the boat in global frame. 0 degrees is East. 90 degrees is North.

    Returns:
       float, float pair representing the measuredWindSpeed (same units as input speed), measuredWindDirectionDegrees
       (0 degrees is wind blowing to the right. 90 degrees is wind blowing forward)
    '''
    globalWindRadians, headingRadians = math.radians(globalWindDirectionDegrees), math.radians(headingDegrees)

    # GF = global frame. BF = boat frame
    # Calculate the measuredWindSpeed in the global frame
    measuredWindXGF = globalWindSpeed * math.cos(globalWindRadians) - boatSpeed * math.cos(headingRadians)
    measuredWindYGF = globalWindSpeed * math.sin(globalWindRadians) - boatSpeed * math.sin(headingRadians)

    # Calculated the measuredWindSpeed in the boat frame
    measuredWindXBF = measuredWindXGF * math.sin(headingRadians) - measuredWindYGF * math.cos(headingRadians)
    measuredWindYBF = measuredWindXGF * math.cos(headingRadians) + measuredWindYGF * math.sin(headingRadians)

    # Convert to speed and direction
    measuredWindDirectionDegrees = math.degrees(math.atan2(measuredWindYBF, measuredWindXBF))
    measuredWindSpeed = (measuredWindYBF**2 + measuredWindXBF**2)**0.5

    return measuredWindSpeed, measuredWindDirectionDegrees


def headingToBearingDegrees(headingDegrees):
    '''Calculates the bearing angle given the heading angle.

    Note: Heading is defined using cartesian coordinates. 0 degrees is East. 90 degrees in North. 270 degrees is South.
          Bearing is defined differently. 0 degrees is North. 90 degrees is East. 180 degrees is South.
          Heading is used for most of this code-based, but bearing is used for interfacing with the geopy module and
          other code external to local-pathfinding.

          Previously, /desired_heading_degrees seems to have been bounded from -90 to 270 degrees, so only the simple
          change below is needed to bound it from 0 to 360 degrees.

    Args:
       headingDegrees (float): heading in degrees

    Returns:
       float representing the same angle in bearing coordinates
    '''
    bearingDegrees = -headingDegrees + 90

    if (bearingDegrees < 0):
        bearingDegrees += 360

    return bearingDegrees


def bearingToHeadingDegrees(bearingDegrees):
    '''Calculates the heading angle given the bearing angle.

    Note: Heading is defined using cartesian coordinates. 0 degrees is East. 90 degrees in North. 270 degrees is South.
          Bearing is defined differently. 0 degrees is North. 90 degrees is East. 180 degrees is South.
          Heading is used for most of this code-based, but bearing is used for interfacing with the geopy module and
          other code external to local-pathfinding.

    Args:
       bearingDegrees (float): heading in degrees

    Returns:
       float representing the same angle in heading coordinates
    '''
    return -bearingDegrees + 90


def vectorAverage(magnitudes, angles, weights=None):
    """Computes the weighted average of vectors using vector addition.

    Args:
        magnitudes (list): The list of magnitudes (floats).
        angles (list): The list of angles (floats in [0, 360]).
        weights (list): The list of weights (floats in [0, 1]).
    Returns:
        float: The magnitude and angle of the vector sum.
    """
    # Compute components of the average vector
    y_components = []
    x_components = []
    for i in range(len(angles)):
        angle = np.deg2rad(angles[i])
        y_components.append(magnitudes[i] * np.sin(angle))
        x_components.append(magnitudes[i] * np.cos(angle))
    y_component = np.average(y_components, weights=weights)
    x_component = np.average(x_components, weights=weights)

    magnitude = np.hypot(y_component, x_component)
    mean_180 = np.rad2deg(np.arctan2(y_component, x_component))  # mean_180 in [-180, 180]
    mean_360 = mean_180 if mean_180 >= 0 or np.isclose(mean_180, 0) else 360 + mean_180  # mean_360 in [0, 360)
    return magnitude, mean_360


def ewma(current_val, past_ewma, weight, is_angle):
    """Calculate the exponentially weighted moving average (EWMA) from the current value and past EWMA:
    https://hackaday.com/2019/09/06/sensor-filters-for-coders/.
    Weight is a discrete linear function of current_val.

    Args:
        current_val (float): Current value.
        past_ewma (float): Past value (EWMA).
        weight (float): The weight of current_val; the weight of past_ewma is (1-weight).
        is_angle (bool): True when getting the field is an angle, false otherwise.

    Returns:
        float: The exponentially weighted moving average.
    """
    if past_ewma is None:
        return current_val

    if is_angle:
        _, current_val = mitsutaVals((past_ewma, current_val))
    return weight * current_val + (1 - weight) * past_ewma


def mitsutaVals(angles):
    """Make angle readings continuous following
    https://journals.ametsoc.org/view/journals/apme/25/10/1520-0450_1986_025_1387_eospeo_2_0_co_2.xml following
    section 2.d. Assumes the time scale is small enough so that the direction to the next reading is < 180.
    Once it is continuous, the mean and standard deviation can be computed using conventional methods.

    Args:
        angles (list): List of floats in [0, 360].

    Returns:
        list: angles such that its values are sequentially continuous.
            Example of readings crossing the discontinuity point:
                mitsutaVals([359.0, 1.0]) = [359, 361.0]
                mitsutaVals([1.0, 359.0]) = [1.0, -1.0]
    """
    vals = [angles[0]]
    for i in range(1, len(angles)):
        delta = angles[i] - angles[i - 1]
        if delta < -PI_DEG:
            delta += 2 * PI_DEG
        elif delta > PI_DEG:
            delta -= 2 * PI_DEG
        vals.append(vals[i - 1] + delta)
    return vals


def isValid(xy, obstacles):
    '''Checks if the given xy is a valid position, given obstacles

    Args:
       xy (list of two floats): xy position of the boat
       obstacles (list of obstacles): List of obstacles that the boat must not be contacting

    Returns:
       bool True iff the boat's xy position does not overlap with any obstacle
    '''
    for obstacle in obstacles:
        if not obstacle.isValid(xy):
            return False
    return True


def pathCostThresholdExceeded(path):
    '''Checks if the given path's cost is above the cost threshold for its length
    Args:
       path (Path): Path object whose cost will be evaluated
    Returns:
       bool True iff the path's cost per km between its start and goal is greater than the threshold
    '''
    latlons = path.getLatlons()
    startLatlon, endLatlon = latlons[0], latlons[len(latlons) - 1]
    distanceApartKm = distance((startLatlon.lat, startLatlon.lon), (endLatlon.lat, endLatlon.lon)).kilometers
    costThreshold = COST_THRESHOLD_PER_KM * distanceApartKm
    pathCost = path.getCost()
    rospy.loginfo("pathCost = {}. Cost threshold for this length = {}"
                  .format(pathCost, costThreshold))
    return pathCost > costThreshold


def getShipsSortedByDistance(ships, sailbotLatlon):
    '''Gets ships sorted by distance to sailbotLatlon
    Args:
       ships (list of sailbot_msg.msg._AISShip.AISShip): Ships to sort by distance
       sailbotLatlon (sailbot_msg.msg._latlon.latlon): Latlon of sailbot to calculate distance from
    Returns:
       list of sailbot_msg.msg._AISShip.AISShip same length as ships sorted by distance to sailbotLatlon
    '''
    distances = [distance((ship.lat, ship.lon), (sailbotLatlon.lat, sailbotLatlon.lon)) for ship in ships]
    return [ship for _, ship in sorted(zip(distances, ships))]


# Smallest signed angle (degrees)
def ssa_deg(angle):
    return ((angle + 180) % 360) - 180


# Get rosparam, or default if empty string
def get_rosparam_or_default_if_invalid(rosparam_name, default, warn_if_empty_string=False, rosparam_type_cast=float):
    rosparam = rospy.get_param(rosparam_name, default=default)

    # If empty string, use default
    if rosparam == '':
        if warn_if_empty_string:
            rospy.logwarn("Invalid {} must be a {}, but received {} = {}"
                          .format(rosparam_name, rosparam_type_cast, rosparam_name, rosparam))
            rospy.logwarn("Defaulting to {} = {}".format(rosparam_name, default))
        return default

    # If not empty string, only use default if invalid rosparam
    try:
        rosparam = rosparam_type_cast(rosparam)
    except ValueError:
        rospy.logwarn("Invalid {} must be a {}, but received {} = {}"
                      .format(rosparam_name, rosparam_type_cast, rosparam_name, rosparam))
        rospy.logwarn("Defaulting to {} = {}".format(rosparam_name, default))
        rosparam = default
    return rosparam
