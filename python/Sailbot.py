#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from local_pathfinding.msg import AIS_msg, GPS, global_path, latlon, wind
from geopy.distance import great_circle

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
        # Update globalPath current one is None
        if self.globalPath is None:
            rospy.loginfo("NEW GLOBAL PATH RECEIVED")
            self.globalPath = data.global_path
            self.globalPathIndex = 0

        # Update globalPath current one different from new one
        oldFirstPoint = (self.globalPath[0].lat, self.globalPath[0].lon)
        newFirstPoint = (data.global_path[0].lat, data.global_path[0].lon)
        if great_circle(oldFirstPoint, newFirstPoint) > 0.001:
            rospy.loginfo("NEW GLOBAL PATH RECEIVED")
            self.globalPath = data.global_path
            self.globalPathIndex = 0


    def __init__(self):
        self.position = latlon() 
        self.windDirection = 0
        self.windSpeed = 0
        self.AISData = AIS_msg()
        self.heading = 0
        self.speed = 0
        self.globalPath = global_path([latlon()]).global_path
        self.globalPathIndex = 0

        rospy.init_node('local_pathfinding', anonymous=True)
        rospy.Subscriber("MOCK_GPS", GPS, self.positionCallback)
        rospy.Subscriber("MOCK_wind", wind, self.windCallback)
        rospy.Subscriber("MOCK_AIS", AIS_msg, self.AISDataCallback)
        rospy.Subscriber("MOCK_global_path", global_path, self.globalPathCallback)


# Example code of how this class works.
if __name__ == '__main__':
    sailbot = Sailbot()
    r = rospy.Rate(1) #hz

    while not rospy.is_shutdown():
        boatState = sailbot.getCurrentState()
        print(boatState)
        print("************************")
        r.sleep()
