#!/usr/bin/env python
from datetime import datetime
from datetime import date
import os
import rospy
import time
import math
from geopy.distance import distance
from std_msgs.msg import Float64
from sailbot_msg.msg import latlon

# Location constants
PORT_RENFREW_LATLON = latlon(48.5, -124.8)
MAUI_LATLON = latlon(20.0, -156.0)

# Constants
PATH_UPDATE_TIME_LIMIT_SECONDS = 7200
AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM = 3.0

# Scale NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES and NUM_LOOK_AHEAD_WAYPOINTS_FOR_UPWIND_DOWNWIND to change based on
# waypoint distance
LOOK_AHEAD_FOR_OBSTACLES_KM = 20
NUM_LOOK_AHEAD_WAYPOINTS_FOR_OBSTACLES = int(math.ceil(LOOK_AHEAD_FOR_OBSTACLES_KM /
                                                       AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM))
LOOK_AHEAD_FOR_UPWIND_DOWNWIND_KM = 10
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

# Constant for number of speedup subscribers needed before publishing initial_speedup
MIN_NUM_SPEEDUP_SUBS_BEFORE_PUBLISHING = 6


def takeScreenshot():
    '''Take a screenshot and save it to a png file.

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


def measuredWindToGlobalWind(measuredWindDirectionDegrees, measuredWindSpeedKmph, boatSpeedKmph, headingDegrees):
    '''Calculate the global wind based on the measured wind and the boat velocity

    Returns:
    float, float pair representing the globalWindSpeed (same units as input speed), globalWindDirectionDegrees
    '''
    measuredWindRadians = math.radians(measuredWindDirectionDegrees)
    headingRadians = math.radians(headingDegrees)

    # GF = global frame. BF = boat frame
    # Calculate wind speed in boat frame. X is right. Y is forward.
    measuredWindSpeedXBF = measuredWindSpeedKmph * math.cos(measuredWindRadians)
    measuredWindSpeedYBF = measuredWindSpeedKmph * math.sin(measuredWindRadians)

    # Assume boat is moving entirely in heading direction, so all boat speed is in boat frame Y direction
    trueWindSpeedXBF = measuredWindSpeedXBF
    trueWindSpeedYBF = measuredWindSpeedYBF + boatSpeedKmph

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
          Heading is used for most of this code-based, but bearing is used for interfacing with the geopy module.

    Args:
       xy (list of two floats): The xy (km) coordinates whose latlon coordinates will be calculated.
       referenceLatlon (sailbot_msg.msg._latlon.latlon): The latlon that will be located at (0,0) wrt xy.

    Returns:
       sailbot_msg.msg._latlon.latlon representing the position of xy in latlon coordinates
    '''
    return -headingDegrees + 90


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


def setInitialSpeedup():
    '''Wait until there are enough speedup subscribers before publishing initial speedup.
    Assumes 1.0 if "initial_speedup" parameter not set.

    Note: Expects that rospy.init_node() has already been called before.
    '''
    initial_speedup = rospy.get_param('initial_speedup', default=1.0)
    speedupPublisher = rospy.Publisher('speedup', Float64, queue_size=4)

    # If initial speedup is about 1.0, don't need to do anything here
    if abs(initial_speedup - 1.0) < 0.1:
        return

    # Wait for other nodes before publishing
    numConnections = speedupPublisher.get_num_connections()
    while numConnections < MIN_NUM_SPEEDUP_SUBS_BEFORE_PUBLISHING:
        rospy.loginfo("{} speedup subscribers found. Waiting for at least {} speedup subscribers "
                      "before publishing initial speedup"
                      .format(numConnections, MIN_NUM_SPEEDUP_SUBS_BEFORE_PUBLISHING))
        time.sleep(1)

        # Calculate number of connections at the end of each loop
        numConnections = speedupPublisher.get_num_connections()

    # Publish message, then must wait to ensure other nodes receive the message before continuing
    rospy.loginfo("{} speedup subscribers found, which is greater than or equal to the required {} speedup subscribers"
                  .format(numConnections, MIN_NUM_SPEEDUP_SUBS_BEFORE_PUBLISHING))
    rospy.loginfo("Publishing initial_speedup = {}".format(initial_speedup))
    speedupPublisher.publish(initial_speedup)
    time.sleep(1)


# Smallest signed angle (degrees)
def ssa_deg(angle):
    return ((angle + 180) % 360) - 180
