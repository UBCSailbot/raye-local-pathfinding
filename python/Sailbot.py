#!/usr/bin/env python
import rospy
from Sailbot_Custom_ROS_Message_Example.msg import AIS
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64

class WindSensorData:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "x: {0}. y: {1}".format(self.x, self.y)

class GPSCoordinates:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def __str__(self):
        return "Lat: {0}. Long: {1}".format(self.latitude, self.longitude)

class BoatState:
    def __init__(self, globalWaypoint, currentPosition, windSensorData, AISData, heading):
        self.globalWaypoint = globalWaypoint
        self.currentPosition = currentPosition
        self.windSensorData = windSensorData
        self.AISData = AISData
        self.heading = heading

    def __str__(self):
        return (("Global waypoint: {0}\n"
                 "Current position: {1}\n"
                 "Wind sensor: {2}\n"
                 "AISData: {3}\n"
                 "Heading: {4}\n")
                 .format(self.globalWaypoint, self.currentPosition, self.windSensorData, self.AISData.MMSI, self.heading))


class Sailbot:

    def getCurrentState(self):
        return BoatState(self.globalWaypoint, self.currentPosition, self.windSensorData, self.AISData, self.heading)

    def currentPositionCallback(self, data):
        rospy.loginfo(data)
        self.currentPosition.latitude = data.x
        self.currentPosition.longitude = data.y
    
    def windSensorDataCallback(self, data):
        rospy.loginfo(data)
        self.windSensorData.x = data.x
        self.windSensorData.y = data.y

    def AISDataCallback(self, data):
        rospy.loginfo(data)
        self.AISData = data

    def headingCallback(self, data):
        rospy.loginfo(data)
        self.heading = data.data

    def __init__(self):
        self.globalWaypoint = GPSCoordinates(0, 0)
        self.currentPosition = GPSCoordinates(0, 0)
        self.windSensorData = WindSensorData(0, 0)
        self.AISData = AIS()
        self.heading = 0

        rospy.init_node('local_pathfinding', anonymous=True)
        rospy.Subscriber("current_position", Pose2D, self.currentPositionCallback)
        rospy.Subscriber("windSensorData", Pose2D, self.windSensorDataCallback)
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
