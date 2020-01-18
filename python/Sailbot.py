#!/usr/bin/env python
import rospy
from local_pathfinding.msg import wind
from local_pathfinding.msg import AIS
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64

class GPSCoordinates:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def __str__(self):
        return "Lat: {0}. Long: {1}".format(self.latitude, self.longitude)

class BoatState:
    def __init__(self, globalWaypoint, currentPosition, wind_direction, wind_speed, AISData, heading):
        self.globalWaypoint = globalWaypoint
        self.currentPosition = currentPosition
        self.wind_direction = wind_direction
        self.wind_speed = wind_speed
        self.AISData = AISData
        self.heading = heading

    def __str__(self):
        return (("Global waypoint: {0}\n"
                 "Current position: {1}\n"
                 "Wind direction: {2}\n"
                 "Wind speed: {3}\n"
                 "AISData: {4}\n"
                 "Heading: {5}\n")
                 .format(self.globalWaypoint, self.currentPosition, self.wind_direction, self.wind_speed, self.AISData.MMSI, self.heading))


class Sailbot:

    def getCurrentState(self):
        return BoatState(self.globalWaypoint, self.currentPosition, self.wind_direction, self.wind_speed, self.AISData, self.heading)

    def currentPositionCallback(self, data):
        rospy.loginfo(data)
        self.currentPosition.latitude = data.x
        self.currentPosition.longitude = data.y
    
    def windCallback(self, data):
        rospy.loginfo(data)
        self.wind_direction = data.direction
        self.wind_speed = data.speed

    def AISDataCallback(self, data):
        rospy.loginfo(data)
        self.AISData = data

    def headingCallback(self, data):
        rospy.loginfo(data)
        self.heading = data.data

    def __init__(self):
        self.globalWaypoint = GPSCoordinates(0, 0)
        self.currentPosition = GPSCoordinates(0, 0)
        self.wind_direction = 0
        self.wind_speed = 0
        self.AISData = AIS()
        self.heading = 0

        rospy.init_node('local_pathfinding', anonymous=True)
        rospy.Subscriber("current_position", Pose2D, self.currentPositionCallback)
        rospy.Subscriber("MOCK_wind", wind, self.windCallback)
        rospy.Subscriber("AISData", AIS, self.AISDataCallback)
        rospy.Subscriber("heading", Float64, self.headingCallback)


# Example code of how this class works.
if __name__ == '__main__':
    sailbot = Sailbot()
    r = rospy.Rate(1) #hz

    while not rospy.is_shutdown():
        boatState = sailbot.getCurrentState()
        print(boatState)
        print("************************")
        r.sleep()
