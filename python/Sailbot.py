#!/usr/bin/env python
import rospy
from local_pathfinding.msg import AISMsg, GPS, path, latlon, windSensor
from geopy.distance import distance

class BoatState:
    def __init__(self, globalWaypoint, position, measuredWindDirectionDegrees, measuredWindSpeedKmph, AISData, headingDegrees, speedKmph):
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
                 "Speed kmph: {6}\n")
                 .format(self.globalWaypoint, self.position, self.measuredWindDirectionDegrees, self.measuredWindSpeedKmph, self.AISData.ships, self.headingDegrees, self.speedKmph))


class Sailbot:
    def getCurrentState(self):
        if self.globalPathIndex >= len(self.globalPath):
            rospy.logwarn("Global path index is out of range: index = {} len(globalPath) = {}".format(self.globalPathIndex, self.globalPath))
            rospy.logwarn("Setting globalWaypoint to be the last element of the globalPath")
            self.globalPathIndex = len(self.globalPath) - 1
        return BoatState(self.globalPath[self.globalPathIndex], self.position, self.measuredWindDirectionDegrees, self.measuredWindSpeedKmph, self.AISData, self.headingDegrees, self.speedKmph)

    def positionCallback(self, data):
        self.position.lat = data.lat
        self.position.lon = data.lon
        self.headingDegrees = data.headingDegrees
        self.speedKmph = data.speedKmph

    def windCallback(self, data):
        self.measuredWindDirectionDegrees = data.measuredDirectionDegrees
        self.measuredWindSpeedKmph = data.measuredSpeedKmph

    def AISDataCallback(self, data):
        self.AISData = data

    def globalPathCallback(self, data):
        # Check that new path is valid
        isValidNewPath = (not data.waypoints is None and len(data.waypoints) >= 2)
        if not isValidNewPath:
            rospy.logwarn("Invalid global path received.")
            return

        # Update if current path is different from new path
        if self.isNewPath(self.globalPath, data.waypoints):
            self.updateGlobalPath(data)
        else:
            rospy.logwarn_once("New global path is the same as the current path. This warning will only show once")
            # rospy.logwarn("New global path is the same as the current path.")


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
        self.globalPathIndex = 1

    def __init__(self, nodeName):
        self.position = latlon(0, 0) 
        self.measuredWindDirectionDegrees = 0
        self.measuredWindSpeedKmph = 0
        self.AISData = AISMsg()
        self.headingDegrees = 0
        self.speedKmph = 0
        self.globalPath = path([self.position, latlon(1, 1)]).waypoints
        self.globalPathIndex = 1  # First waypoint is the start point, so second waypoint is the next global waypoint
        self.newGlobalPathReceived = False

        rospy.init_node(nodeName, anonymous=True)
        rospy.Subscriber("GPS", GPS, self.positionCallback)
        rospy.Subscriber("windSensor", windSensor, self.windCallback)
        rospy.Subscriber("AIS", AISMsg, self.AISDataCallback)
        rospy.Subscriber("globalPath", path, self.globalPathCallback)


# Example code of how this class works.
if __name__ == '__main__':
    sailbot = Sailbot(nodeName='testSailbot')
    r = rospy.Rate(1) #hz

    while not rospy.is_shutdown():
        boatState = sailbot.getCurrentState()
        print(boatState)
        print("************************")
        r.sleep()
