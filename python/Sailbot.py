#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from local_pathfinding.msg import AIS_msg, GPS, path, latlon, wind
from geopy.distance import distance

class BoatState:
    def __init__(self, globalWaypoint, position, windDirection, windSpeed, AISData, heading, speed):
        self.globalWaypoint = globalWaypoint
        self.position = position
        self.windDirection = windDirection
        self.windSpeed = windSpeed
        self.AISData = AISData
        self.heading = heading
        self.speed = speed

    # AISData currently printing out a list of all ships and their attributes
    def __str__(self):
        return (("Global waypoint: {0}\n"
                 "Current position: {1}\n"
                 "Wind direction: {2}\n"
                 "Wind speed: {3}\n"
                 "AISData: {4}\n"
                 "Heading: {5}\n"
                 "Speed: {6}\n")
                 .format(self.globalWaypoint, self.position, self.windDirection, self.windSpeed, self.AISData.ships, self.heading, self.speed))


class Sailbot:
    def getCurrentState(self):
        if self.globalPathIndex >= len(self.globalPath):
            rospy.loginfo("Global path index is out of range: index = {} len(globalPath) = {}".format(self.globalPathIndex, self.globalPath))
            rospy.loginfo("Setting localWaypoint to be the last element of the localPath")
            self.globalPathIndex = len(self.globalPath) - 1
        return BoatState(self.globalPath[self.globalPathIndex], self.position, self.windDirection, self.windSpeed, self.AISData, self.heading, self.speed)

    def positionCallback(self, data):
        self.position.lat = data.lat
        self.position.lon = data.lon
        self.heading = data.heading
        self.speed = data.speed

    def windCallback(self, data):
        self.windDirection = data.direction
        self.windSpeed = data.speed

    def AISDataCallback(self, data):
        self.AISData = data

    def globalPathCallback(self, data):
        # Check that new path is valid
        isValidNewPath = (not data.path is None and len(data.path) >= 2)
        if not isValidNewPath:
            rospy.loginfo("Invalid global path received.")
            return

        # Update globalPath if current path is None
        if self.globalPath is None:
            updateGlobalPath(data)
            return

        # Update globalPath if current path different from new path
        oldFirstPoint = (self.globalPath[0].lat, self.globalPath[0].lon)
        newFirstPoint = (data.path[0].lat, data.path[0].lon)
        if distance(oldFirstPoint, newFirstPoint).kilometers > 1:
            updateGlobalPath(data)
        else:
            rospy.loginfo("New global path is the same as the current path.")

    def updateGlobalPath(self, data):
        rospy.loginfo("New global path received.")
        self.newGlobalPathReceived = True
        self.globalPath = data.path
        self.globalPathIndex = 1

    def __init__(self):
        self.position = latlon(0, 0) 
        self.windDirection = 0
        self.windSpeed = 0
        self.AISData = AIS_msg()
        self.heading = 0
        self.speed = 0
        self.globalPath = path([self.position, latlon(1, 1)]).path
        self.globalPathIndex = 1  # First waypoint is the start point, so second waypoint is the next global waypoint
        self.newGlobalPathReceived = False

        rospy.init_node('local_pathfinding', anonymous=True)
        rospy.Subscriber("MOCK_GPS", GPS, self.positionCallback)
        rospy.Subscriber("MOCK_wind", wind, self.windCallback)
        rospy.Subscriber("MOCK_AIS", AIS_msg, self.AISDataCallback)
        rospy.Subscriber("MOCK_global_path", path, self.globalPathCallback)


# Example code of how this class works.
if __name__ == '__main__':
    sailbot = Sailbot()
    r = rospy.Rate(1) #hz

    while not rospy.is_shutdown():
        boatState = sailbot.getCurrentState()
        print(boatState)
        print("************************")
        r.sleep()
