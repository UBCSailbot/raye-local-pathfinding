#!/usr/bin/env python
import rospy
from sailbot_msg.msg import AISMsg, GPS, path, latlon, windSensor
from geopy.distance import distance
import time
import sys


class BoatState:
    def __init__(self, globalWaypoint, position, measuredWindDirectionDegrees, measuredWindSpeedKmph, AISData,
                 headingDegrees, speedKmph):
        self.globalWaypoint = globalWaypoint
        self.position = position
        self.measuredWindDirectionDegrees = measuredWindDirectionDegrees
        self.measuredWindSpeedKmph = measuredWindSpeedKmph
        self.AISData = AISData
        self.headingDegrees = headingDegrees
        self.speedKmph = speedKmph

    # AISData currently printing out a list of all ships and their attributes
    def __str__(self):
        return (("Global waypoint: {0}\n"
                 "Current position: {1}\n"
                 "Wind direction degrees: {2}\n"
                 "Wind speed kmph: {3}\n"
                 "AISData: {4}\n"
                 "Heading degrees: {5}\n"
                 "Speed kmph: {6}\n") .format(self.globalWaypoint, self.position, self.measuredWindDirectionDegrees,
                                              self.measuredWindSpeedKmph, self.AISData.ships, self.headingDegrees,
                                              self.speedKmph))


class Sailbot:
    def getCurrentState(self):
        self.printWarningsIfStaleData()
        if self.globalPathIndex >= len(self.globalPath):
            rospy.logwarn("Global path index is out of range: index = {} len(globalPath) = {}"
                          .format(self.globalPathIndex, len(self.globalPath)))
            rospy.logwarn("Setting globalWaypoint to be the last element of the globalPath")
            self.globalPathIndex = len(self.globalPath) - 1
        return BoatState(self.globalPath[self.globalPathIndex], self.position, self.measuredWindDirectionDegrees,
                         self.measuredWindSpeedKmph, self.AISData, self.headingDegrees, self.speedKmph)

    def GPSCallback(self, data):
        self.position = latlon(data.lat, data.lon)
        self.headingDegrees = data.headingDegrees
        self.speedKmph = data.speedKmph
        self.GPSLastUpdate = time.time()

    def windSensorCallback(self, data):
        self.measuredWindDirectionDegrees = data.measuredDirectionDegrees
        self.measuredWindSpeedKmph = data.measuredSpeedKmph
        self.windSensorLastUpdate = time.time()

    def AISCallback(self, data):
        self.AISData = data
        self.AISLastUpdate = time.time()

    def globalPathCallback(self, data):
        # Check that new path is valid
        isValidNewPath = (data.waypoints is not None and len(data.waypoints) >= 2)
        if not isValidNewPath:
            rospy.logwarn("Invalid global path received.")
            return

        # Update if current path is different from new path
        if self.isNewPath(self.globalPath, data.waypoints):
            self.updateGlobalPath(data)
        else:
            rospy.logwarn_once("New global path is the same as the current path. This warning will only show once")
            # rospy.logwarn("New global path is the same as the current path.")

        self.globalPathLastUpdate = time.time()

    def isNewPath(self, oldPath, newPath):
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

    def updateGlobalPath(self, data):
        rospy.loginfo("New global path received.")
        self.newGlobalPathReceived = True
        self.globalPath = data.waypoints
        self.globalPathIndex = 1  # First waypoint is the start point, so second waypoint is the next global waypoint

    def printWarningsIfStaleData(self):
        current_time = time.time()
        GPSStaleLimitSeconds = 60
        windSensorStaleLimitSeconds = 60
        AISStaleLimitSeconds = 60
        globalPathStaleLimitSeconds = 60 * 60 * 24 * 2
        if current_time - self.GPSLastUpdate > GPSStaleLimitSeconds:
            rospy.logwarn("GPS stale. Not updated for {} seconds ({} limit)"
                          .format(current_time - self.GPSLastUpdate, GPSStaleLimitSeconds))
        if current_time - self.windSensorLastUpdate > windSensorStaleLimitSeconds:
            rospy.logwarn("windSensor stale. Not updated for {} seconds ({} limit)"
                          .format(current_time - self.windSensorLastUpdate, windSensorStaleLimitSeconds))
        if current_time - self.AISLastUpdate > AISStaleLimitSeconds:
            rospy.logwarn("AIS stale. Not updated for {} seconds ({} limit)"
                          .format(current_time - self.AISLastUpdate, AISStaleLimitSeconds))
        if current_time - self.globalPathLastUpdate > globalPathStaleLimitSeconds:
            rospy.logwarn("globalPath stale. Not updated for {} seconds ({} limit)"
                          .format(current_time - self.globalPathLastUpdate, globalPathStaleLimitSeconds))

    def waitForFirstSensorDataAndGlobalPath(self):
        def isMissingFirstSensorDataOrGlobalPath():
            return (self.position is None or self.measuredWindDirectionDegrees is None or
                    self.measuredWindSpeedKmph is None or self.AISData is None or
                    self.headingDegrees is None or self.speedKmph is None or self.globalPath is None)

        while isMissingFirstSensorDataOrGlobalPath():
            # Exit if shutdown
            if rospy.is_shutdown():
                rospy.loginfo("rospy.is_shutdown() is True. Exiting")
                sys.exit()
            else:
                rospy.loginfo("Waiting for sailbot to receive first sensor data and global path")
                rospy.loginfo("self.position is None? {}. self.measuredWindDirectionDegrees is None? {}. "
                              "self.measuredWindSpeedKmph is None? {}. self.AISData is None? {}. "
                              "self.headingDegrees is None? {}. self.speedKmph is None? {}. "
                              "self.globalPath is None? {}."
                              .format(self.position is None, self.measuredWindDirectionDegrees is None,
                                      self.measuredWindSpeedKmph is None, self.AISData is None,
                                      self.headingDegrees is None, self.speedKmph is None, self.globalPath is None))
                time.sleep(1)

        rospy.loginfo("First sensor data and global path received")

    def __init__(self, nodeName):
        # Sensor data and global path
        self.position = None
        self.measuredWindDirectionDegrees = None
        self.measuredWindSpeedKmph = None
        self.AISData = None
        self.headingDegrees = None
        self.speedKmph = None
        self.globalPath = None

        # Sensor data and global path last received time
        self.GPSLastUpdate = None
        self.windSensorLastUpdate = None
        self.AISLastUpdate = None
        self.globalPathLastUpdate = None

        # State of sailboat (to be interacted with in main_loop.py)
        self.globalPathIndex = -1
        self.newGlobalPathReceived = False

        rospy.init_node(nodeName, anonymous=True)
        rospy.Subscriber("GPS", GPS, self.GPSCallback)
        rospy.Subscriber("windSensor", windSensor, self.windSensorCallback)
        rospy.Subscriber("AIS", AISMsg, self.AISCallback)
        rospy.Subscriber("globalPath", path, self.globalPathCallback)


# Example code of how this class works.
if __name__ == '__main__':
    sailbot = Sailbot(nodeName='testSailbot')
    r = rospy.Rate(1)  # hz

    while not rospy.is_shutdown():
        boatState = sailbot.getCurrentState()
        print(boatState)
        print("************************")
        r.sleep()
