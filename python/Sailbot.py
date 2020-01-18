#!/usr/bin/env python
import rospy
from local_pathfinding.msg import wind
from local_pathfinding.msg import AIS
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
from local_pathfinding.msg import AIS_msg, GPS, global_path, latlon

class GPSCoordinates:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def __str__(self):
        return "Lat: {0}. Long: {1}".format(self.latitude, self.longitude)

class BoatState:
    def __init__(self, globalWaypoint, currentPosition, wind_direction, wind_speed, AISData, heading, speed):
        self.globalWaypoint = globalWaypoint
        self.currentPosition = currentPosition
        self.wind_direction = wind_direction
        self.wind_speed = wind_speed
        self.AISData = AISData
        self.heading = heading
        self.speed = speed

    #AISData currently printing out a list of all ships and their attributes
    def __str__(self):
        return (("Global waypoint: {0}\n"
                 "Current position: {1}\n"
                 "Wind direction: {2}\n"
                 "Wind speed: {3}\n"
                 "AISData: {4}\n"
                 "Heading: {5}\n"
                 "Speed: {6}\n")
                 .format(self.globalWaypoint, self.currentPosition, self.wind_direction, self.wind_speed, self.AISData.ships, self.heading, self.speed))


class Sailbot:

    def getCurrentState(self):
        return BoatState(self.globalWaypoint, self.currentPosition, self.wind_direction, self.wind_speed, self.AISData, self.heading, self.speed)

    def currentPositionCallback(self, data):
        self.currentPosition.latitude = data.lat
        self.currentPosition.longitude = data.lon
        self.heading = data.heading
        self.speed = data.speed

    def windCallback(self, data):
        self.wind_direction = data.direction
        self.wind_speed = data.speed

    def AISDataCallback(self, data):
        self.AISData = data

    def globalPathCallback(self, data):
        self.globalWaypoint.latitude = data.global_path[0].lat
        self.globalWaypoint.longitude = data.global_path[0].lon

    def __init__(self):
        self.globalWaypoint = GPSCoordinates(0, 0)
        self.currentPosition = GPSCoordinates(0, 0)
        self.wind_direction = 0
        self.wind_speed = 0
        self.AISData = AIS_msg()
        self.heading = 0
        self.speed = 0

        rospy.init_node('local_pathfinding', anonymous=True)
        rospy.Subscriber("MOCK_GPS", GPS, self.currentPositionCallback)
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
