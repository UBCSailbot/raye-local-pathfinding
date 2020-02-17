#!/usr/bin/env python
import rospy
import math
import random
from geopy.distance import distance
from utilities import headingToBearingDegrees

import local_pathfinding.msg as msg

class Ship:
    def __init__(self, id, sailbot_lat, sailbot_lon, publishPeriodSeconds):
        self.id = id
        # Set boat position to be within 25 km radius from the given latlon
        boatLatlon = distance(kilometers=random.randint(0, 25)).destination(point=(sailbot_lat, sailbot_lon), bearing=random.randint(0, 360))
        self.lat = boatLatlon.latitude
        self.lon = boatLatlon.longitude
        self.headingDegrees = random.randint(0, 360)
        self.speedKmph = random.randint(0, 15)

        self.publishPeriodSeconds = publishPeriodSeconds

    def move(self):
        distanceTraveled = distance(kilometers=self.speedKmph * self.publishPeriodSeconds / 3600)
        boatLatlon = distanceTraveled.destination(point=(self.lat, self.lon), bearing=headingToBearingDegrees(self.headingDegrees))

        self.lon = boatLatlon.longitude
        self.lat = boatLatlon.latitude

    def make_ros_message(self):
        return msg.AISShip(
                    self.id,
                    self.lat,
                    self.lon,
                    self.headingDegrees,
                    self.speedKmph)


class MOCK_AISEnvironment: 
    # Just a class to keep track of the ships surrounding the sailbot
    def __init__(self, lat, lon):
        self.publishPeriodSeconds = 1.0
        self.ships = []
        for i in range(10):
            self.ships.append(Ship(i, lat, lon, self.publishPeriodSeconds))

        rospy.init_node('MOCK_AIS', anonymous=True)
        self.publisher = rospy.Publisher("AIS", msg.AISMsg, queue_size=4)

    def move_ships(self):
        for i in range(10):
            self.ships[i].move()

    def make_ros_message(self):
        ship_list = []
        for i in range(10):
            ship_list.append(self.ships[i].make_ros_message())
        return msg.AISMsg(ship_list)

if __name__ == '__main__':
    # Create boats near Port Renfrew: 48.5, -124.8
    ais_env = MOCK_AISEnvironment(48.5, -124.8)
    r = rospy.Rate(1.0 / ais_env.publishPeriodSeconds) #hz

    while not rospy.is_shutdown():
        ais_env.move_ships()
        data = ais_env.make_ros_message()
        ais_env.publisher.publish(data)
        r.sleep()
