#!/usr/bin/env python
import rospy
import math
import random

import local_pathfinding.msg as msg
import geopy.distance

class MOCK_ControllerAndSailbot: 
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
        self.heading = math.radians(180)
        self.speed = 0
        self.publish_period = 1 # Seconds

        rospy.init_node('Sailbot_Listener', anonymous=True)
        self.publisher = rospy.Publisher("MOCK_GPS", msg.GPS, queue_size=4)
        rospy.Subscriber("MOCK_desired_heading", msg.desired_heading, self.desiredHeadingCallback)

    def move(self):
        # TODO: Change this so that the boat
        # moves at a reasonable speed
        distance_per_period = 5  # km
        distance = geopy.distance.distance(kilometers = distance_per_period)

        # Heading units: 0 degrees => EAST. 90 degrees => NORTH.
        # Bearing units: 0 degrees => NORTH. 90 degrees => EAST.
        destination = distance.destination(point=(self.lat, self.lon), bearing=-self.heading + 90)

        # TODO: translate to m/s
        self.speed = distance_per_period / self.publish_period

        self.lon = destination.longitude
        self.lat = destination.latitude


    def desiredHeadingCallback(self, data):
        rospy.loginfo(data)
        self.heading = data.heading + random.gauss(0, 0.1)

if __name__ == '__main__':
    MOCK_ctrl_sailbot = MOCK_ControllerAndSailbot(48.5, -124.8)
    r = rospy.Rate(float(1) / MOCK_ctrl_sailbot.publish_period) #hz

    while not rospy.is_shutdown():
        MOCK_ctrl_sailbot.move()
        data = msg.GPS()
        data.lat = MOCK_ctrl_sailbot.lat
        data.lon = MOCK_ctrl_sailbot.lon
        data.heading = MOCK_ctrl_sailbot.heading
        data.speed = MOCK_ctrl_sailbot.speed
        MOCK_ctrl_sailbot.publisher.publish(data)
        r.sleep()
