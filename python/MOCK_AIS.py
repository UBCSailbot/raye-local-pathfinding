#!/usr/bin/env python
import rospy
import math
import random
from geopy.distance import distance
from utilities import headingToBearingDegrees, PORT_RENFREW_LATLON

import local_pathfinding.msg as msg

# Constants
AIS_PUBLISH_PERIOD_SECONDS = 1.0

class Ship:
    def __init__(self, id, sailbot_lat, sailbot_lon, publishPeriodSeconds, speedup):
        self.id = id
        self.headingDegrees = random.randint(0, 360)
        self.speedKmph = random.randint(0, 15)
        self.speedup = speedup

        # Set AIS boat position to be in about 50km radius around sailbot
        boatLatlon = distance(kilometers=abs(random.gauss(10, 5))).destination(point=(sailbot_lat, sailbot_lon), bearing=random.randint(0, 360))
        self.lat = boatLatlon.latitude
        self.lon = boatLatlon.longitude

        self.publishPeriodSeconds = publishPeriodSeconds

    def move(self):
        # Travel greater distance with speedup
        distanceTraveled = distance(kilometers=self.speedKmph * self.publishPeriodSeconds / 3600 * self.speedup)
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
    def __init__(self, lat, lon, speedup):
        self.publishPeriodSeconds = AIS_PUBLISH_PERIOD_SECONDS
        self.numShips = 10
        self.ships = []
        for i in range(self.numShips):
            self.ships.append(Ship(i, lat, lon, self.publishPeriodSeconds, speedup))

        rospy.init_node('MOCK_AIS', anonymous=True)
        self.publisher = rospy.Publisher("AIS", msg.AISMsg, queue_size=4)

    def move_ships(self):
        for i in range(self.numShips):
            self.ships[i].move()

    def make_ros_message(self):
        ship_list = []
        for i in range(self.numShips):
            ship_list.append(self.ships[i].make_ros_message())
        return msg.AISMsg(ship_list)

if __name__ == '__main__':
    # Get speedup parameter
    speedup = rospy.get_param('speedup', 1.0)

    ais_env = MOCK_AISEnvironment(PORT_RENFREW_LATLON.lat, PORT_RENFREW_LATLON.lon, speedup)
    r = rospy.Rate(1.0 / ais_env.publishPeriodSeconds) #hz

    while not rospy.is_shutdown():
        ais_env.move_ships()
        data = ais_env.make_ros_message()
        ais_env.publisher.publish(data)
        r.sleep()
