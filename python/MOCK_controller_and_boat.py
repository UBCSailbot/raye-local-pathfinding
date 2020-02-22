#!/usr/bin/env python
import rospy
import math
import random

import local_pathfinding.msg as msg
import geopy.distance
from utilities import headingToBearingDegrees

class MOCK_ControllerAndSailbot: 
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
        self.headingDegrees = 180
        self.speedKmph = 14.4  # Boat should move at about 4m/s = 14.4 km/h
        self.publishPeriodSeconds = 1.0

        rospy.init_node('MOCK_Sailbot_Listener', anonymous=True)
        self.publisher = rospy.Publisher("GPS", msg.GPS, queue_size=4)
        rospy.Subscriber("desiredHeading", msg.heading, self.desiredHeadingCallback)

    def move(self):
        kmTraveledPerPeriod = self.speedKmph * self.publishPeriodSeconds / 3600.0
        distanceTraveled = geopy.distance.distance(kilometers = kmTraveledPerPeriod)
        destination = distanceTraveled.destination(point=(self.lat, self.lon), bearing=headingToBearingDegrees(self.headingDegrees))

        self.lon = destination.longitude
        self.lat = destination.latitude


    def desiredHeadingCallback(self, data):
        rospy.loginfo(data)
        self.headingDegrees = data.headingDegrees + random.gauss(0, 1)

if __name__ == '__main__':
    MOCK_ctrl_sailbot = MOCK_ControllerAndSailbot(48.5, -124.8)
    r = rospy.Rate(float(1) / MOCK_ctrl_sailbot.publishPeriodSeconds)

    while not rospy.is_shutdown():
        MOCK_ctrl_sailbot.move()
        data = msg.GPS()
        data.lat = MOCK_ctrl_sailbot.lat
        data.lon = MOCK_ctrl_sailbot.lon
        data.headingDegrees = MOCK_ctrl_sailbot.headingDegrees
        data.speedKmph = MOCK_ctrl_sailbot.speedKmph
        MOCK_ctrl_sailbot.publisher.publish(data)
        r.sleep()
