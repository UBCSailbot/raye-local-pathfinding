#!/usr/bin/env python
import rospy
import handleInvalidState as his
from sailbot_msg.msg import AISMsg, GPS, path, latlon, globalWind
from geopy.distance import distance
import time
import sys
import utilities as utils

# Parameters
MAX_NUM_AIS_SHIPS = 50  # Limit number of AIS ships used in full pipeline, makes large performance difference
AIS_SHIPS_IGNORE_LIST = []  # List of ID's of AIS ships to ignore

# Global variables for moving waypoint
goalWasInvalid = False
movedGlobalWaypoint = None


class BoatState:
    """Datatype representing the current state of the sailbot, which includes all information relevant for pathfinding.

    Attributes:
        position (sailbot_msg.msg._latlon.latlon): latlon that is the current latlon position of the boat
        globalWindDirectionDegrees (float): direction that the wind is going
                                            (0 degrees = going east, 90 degrees = going north)
        globalWindSpeedKmph (float): global wind speed
        AISData (sailbot_msg.msg._AISMsg.AISMsg): AIS message with information about nearby boats
        headingDegrees (float): direction that the boat is heading towards
                                (0 degrees = going east, 90 degrees = going north)
        speedKmph (float): speed that the boat is moving
        globalWaypoint (sailbot_msg.msg._latlon.latlon): global waypoint latlon that the sailbot should be going towards
    """
    def __init__(self, globalWaypoint, position, globalWindDirectionDegrees, globalWindSpeedKmph, AISData,
                 headingDegrees, speedKmph):
        self.globalWaypoint = globalWaypoint
        self.position = position
        self.AISData = AISData
        self.headingDegrees = headingDegrees
        self.speedKmph = speedKmph
        self.globalWindDirectionDegrees = globalWindDirectionDegrees
        self.globalWindSpeedKmph = globalWindSpeedKmph

    # AISData currently printing out a list of all ships and their attributes
    def __str__(self):
        return (("Global waypoint: {0}\n"
                 "Current position: {1}\n"
                 "Global wind direction degrees: {2}\n"
                 "Global wind speed kmph: {3}\n"
                 "AISData: {4}\n"
                 "Heading degrees: {5}\n"
                 "Speed kmph: {6}\n") .format(self.globalWaypoint, self.position, self.globalWindDirectionDegrees,
                                              self.globalWindSpeedKmph, self.AISData.ships, self.headingDegrees,
                                              self.speedKmph))


class Sailbot:
    """Class for storing, updating, and maintaining the state of the sailbot.

    Attributes:
        position (sailbot_msg.msg._latlon.latlon): latlon that is the current latlon position of the boat
        measuredWindDirectionDegrees (float): measured direction that the wind is going towards wrt the boat
                                              (0 degrees = going to right of boat, 90 degrees = going to front of boat)
        measuredWindSpeedKmph (float): measured speed of the wind wrt to boat
        AISData (sailbot_msg.msg._AISMsg.AISMsg): AIS message with information about nearby boats
        headingDegrees (float): direction that the boat is heading towards
                                (0 degrees = going east, 90 degrees = going north)
        speedKmph (float): speed that the boat is moving
        globalPath (sailbot_msg.msg._latlon.latlon[]): global path list of latlon waypoints that sailbot should follow

        GPSLastUpdate (float): time.time() when the GPS was last updated
        globalWindLastUpdate (float): time.time() when the global wind was last updated
        AISLastUpdate (float): time.time() when the AIS was last updated
        globalPathLastUpdate (float): time.time() when the global path was last updated

        globalPathIndex (int): index of the global path waypoint (in globalPath) that the sailbot should be going
                               towards. As of right now, has to be updated manually from outside of Sailbot when reached
                               TODO: Could make this able to be updated internally, not need to modify from outside
        newGlobalPathReceived (bool): defaults false. True when a new global path has been received. Can be modified
                                      from outside of Sailbot when newly received global path has been processed
    """
    def __init__(self, nodeName):
        # Sensor data and global path
        self.position = None
        self.AISData = None
        self.headingDegrees = None
        self.speedKmph = None
        self.globalPath = None
        self.globalWindDirectionDegrees = None
        self.globalWindSpeedKmph = None

        # Sensor data and global path last received time
        self.GPSLastUpdate = None
        self.globalWindLastUpdate = None
        self.AISLastUpdate = None
        self.globalPathLastUpdate = None

        # State of sailboat (to be interacted with in main_loop.py)
        self.globalPathIndex = -1
        self.newGlobalPathReceived = False

        rospy.init_node(nodeName, anonymous=True)
        rospy.Subscriber("GPS", GPS, self.GPSCallback)
        rospy.Subscriber("global_wind", globalWind, self.globalWindCallback)
        rospy.Subscriber("AIS", AISMsg, self.AISCallback)
        rospy.Subscriber("globalPath", path, self.globalPathCallback)

    def getCurrentState(self):
        '''Get the current sailbot boatstate, which includes all information relevant for pathfinding.

        Notes:
        * Should only be run after running waitForFirstSensorDataAndGlobalPath() to get valid data, else returns None
        * Prints warnings if there is stale data.
        * If sailbot reaches end of its global path so that its globalPathIndex >= len(globalPath), then it
          simply sets the next global waypoint to be the last global waypoint

        Returns:
           BoatState representing the current state of the sailbot.
           Returns None if first subscriber messages have not been received
        '''
        if self.isMissingFirstSensorDataOrGlobalPath():
            rospy.logerr("Can't get current state: missing first sensor data or global path")
            return None

        # Print warnings about stale data
        self.printWarningsIfStaleData()

        # End of global path edge case
        if self.globalPathIndex >= len(self.globalPath):
            rospy.logwarn("Global path index is out of range: index = {} len(globalPath) = {}"
                          .format(self.globalPathIndex, len(self.globalPath)))
            rospy.logwarn("Setting globalWaypoint to be the last element of the globalPath")
            self.globalPathIndex = len(self.globalPath) - 1

        state = BoatState(self.globalPath[self.globalPathIndex], self.position, self.globalWindDirectionDegrees,
                          self.globalWindSpeedKmph, self.AISData, self.headingDegrees, self.speedKmph)

        # Move global waypoint forward until valid
        global goalWasInvalid
        global movedGlobalWaypoint
        if not goalWasInvalid and not his.checkGoalValidity(state):
            rospy.logwarn("Goal state invalid, finding new global waypoint")
            rospy.loginfo("Old waypoint: {}".format(state.globalWaypoint))

            # calculating waypoint to use in moveGlobalWaypointUntilValid
            isLast = self.globalPathIndex == len(self.globalPath) - 1
            otherWaypointIndex = self.globalPathIndex - 1 if isLast else self.globalPathIndex + 1
            otherWaypoint = self.globalPath[otherWaypointIndex]

            movedGlobalWaypoint = his.moveGlobalWaypointUntilValid(state, isLast, otherWaypoint)
            rospy.loginfo("Moved waypoint: {}".format(movedGlobalWaypoint))
            goalWasInvalid = True

        if goalWasInvalid:
            state.globalWaypoint = movedGlobalWaypoint

        # Expects AISData from /AIS to have heading convention (0 North, 90 East)
        # Convert AIS bearing (0 North, 90 East) to heading (0 East, 90 North)
        for ship in state.AISData.ships:
            ship.headingDegrees = utils.bearingToHeadingDegrees(ship.headingDegrees)

        # If too many AIS ships, keep the closest ones
        if len(state.AISData.ships) > MAX_NUM_AIS_SHIPS:
            shipsSortedByDistance = utils.getShipsSortedByDistance(state.AISData.ships, state.position)
            state.AISData.ships = shipsSortedByDistance[:MAX_NUM_AIS_SHIPS]

        return state

    def GPSCallback(self, data):
        self.position = latlon(data.lat, data.lon)
        self.headingDegrees = data.headingDegrees
        self.speedKmph = data.speedKmph
        self.GPSLastUpdate = time.time()

    def globalWindCallback(self, data):
        self.globalWindDirectionDegrees = data.directionDegrees
        self.globalWindSpeedKmph = data.speedKmph
        self.globalWindLastUpdate = time.time()

    def AISCallback(self, data):
        # handle ships with invalid latlons
        filteredShips = []
        invalidShips = []
        for ship in data.ships:
            if -90 < ship.lat < 90 and -180 < ship.lon < 180:
                filteredShips.append(ship)
            else:
                invalidShips.append(ship)
        if invalidShips:
            warn_msg = "Invalid coordinates for AIS Ships" + \
                ''.join(['\n\tID {}: ({}, {})'.format(ship.ID, ship.lat, ship.lon) for ship in invalidShips])
            rospy.logwarn_throttle(5, warn_msg)

        # ignore ships
        filteredShips = [ship for ship in filteredShips if ship.ID not in AIS_SHIPS_IGNORE_LIST]

        self.AISData = AISMsg(ships=filteredShips)
        self.AISLastUpdate = time.time()

    def globalPathCallback(self, data):
        '''Updates the global path if:

        * The new path is valid (has length >= 2)
        * The new path is different from the old path

        Assumes that global path is setup so that waypoint index 0 is close to the current position and
        waypoint index 1 is the next global waypoint.
        Sets newGlobalPathReceived to True so that external systems can check when a new global path has been received
        '''
        def isNewPath(oldPath, newPath):
            # Check if they are obviously different
            if oldPath is None or not len(oldPath) == len(newPath):
                return True

            # Path is different if there exists a point with distance >1km away from current point
            MAX_DISTANCE_BETWEEN_SAME_POINTS_KM = 1
            for i in range(len(oldPath)):
                oldFirstPoint = (oldPath[i].lat, oldPath[i].lon)
                newFirstPoint = (newPath[i].lat, newPath[i].lon)
                if distance(oldFirstPoint, newFirstPoint).kilometers > MAX_DISTANCE_BETWEEN_SAME_POINTS_KM:
                    return True

            return False

        # Check that new path is valid
        isValidNewPath = (data.waypoints is not None and len(data.waypoints) >= 2)
        if not isValidNewPath:
            rospy.logwarn("Invalid global path received.")
            return

        # Update if current path is different from new path
        if isNewPath(self.globalPath, data.waypoints):
            rospy.loginfo("New global path received.")
            self.newGlobalPathReceived = True
            self.globalPath = data.waypoints
            self.globalPathIndex = 1  # First waypoint is start point, so second waypoint is next global waypoint

        self.globalPathLastUpdate = time.time()

    def printWarningsIfStaleData(self):
        """Prints warning messages if there is stale data (subscribers haven't received new messages)"""
        # Edge case check to avoid null pointer errors
        if self.isMissingFirstSensorDataOrGlobalPath():
            rospy.logwarn("Can't print warnings about stale data: missing first sensor data or global path")
            return

        current_time = time.time()
        GPSStaleLimitSeconds = 60
        globalWindStaleLimitSeconds = 60
        AISStaleLimitSeconds = 60
        globalPathStaleLimitSeconds = 60
        if current_time - self.GPSLastUpdate > GPSStaleLimitSeconds:
            rospy.logwarn("GPS stale. Not updated for {} seconds ({} limit)"
                          .format(current_time - self.GPSLastUpdate, GPSStaleLimitSeconds))
        if current_time - self.globalWindLastUpdate > globalWindStaleLimitSeconds:
            rospy.logwarn("windSensor stale. Not updated for {} seconds ({} limit)"
                          .format(current_time - self.globalWindLastUpdate, globalWindStaleLimitSeconds))
        if current_time - self.AISLastUpdate > AISStaleLimitSeconds:
            rospy.logwarn("AIS stale. Not updated for {} seconds ({} limit)"
                          .format(current_time - self.AISLastUpdate, AISStaleLimitSeconds))
        if current_time - self.globalPathLastUpdate > globalPathStaleLimitSeconds:
            rospy.logwarn("globalPath stale. Not updated for {} seconds ({} limit)"
                          .format(current_time - self.globalPathLastUpdate, globalPathStaleLimitSeconds))

    def waitForFirstSensorDataAndGlobalPath(self):
        """Waits until first sensor data and global path have been received by subscribers"""
        while self.isMissingFirstSensorDataOrGlobalPath():
            # Exit if shutdown
            if rospy.is_shutdown():
                rospy.loginfo("rospy.is_shutdown() is True. Exiting")
                sys.exit()
            else:
                rospy.loginfo("Waiting for sailbot to receive first sensor data and global path")
                rospy.loginfo("self.position is None? {}. self.globalWindDirectionDegrees is None? {}. "
                              "self.globalWindSpeedKmph is None? {}. self.AISData is None? {}. "
                              "self.headingDegrees is None? {}. self.speedKmph is None? {}. "
                              "self.globalPath is None? {}."
                              .format(self.position is None, self.globalWindDirectionDegrees is None,
                                      self.globalWindSpeedKmph is None, self.AISData is None,
                                      self.headingDegrees is None, self.speedKmph is None, self.globalPath is None))
                time.sleep(1)

        rospy.loginfo("First sensor data and global path received")

    def isMissingFirstSensorDataOrGlobalPath(self):
        """Checks if the subscribers have all received their first messages

        Returns:
           bool True iff the subscribers have all received their first messages
        """
        return (self.position is None or self.globalWindDirectionDegrees is None or
                self.globalWindSpeedKmph is None or self.AISData is None or
                self.headingDegrees is None or self.speedKmph is None or self.globalPath is None)


# Example code of how this class works.
if __name__ == '__main__':
    sailbot = Sailbot(nodeName='testSailbot')
    sailbot.waitForFirstSensorDataAndGlobalPath()
    r = rospy.Rate(1)  # hz

    while not rospy.is_shutdown():
        boatState = sailbot.getCurrentState()
        print(boatState)
        print("************************")
        r.sleep()
