#!/usr/bin/env python
import rospy
import math
import random

import local_pathfinding.msg as msg

class Ship:
    def __init__(self, id, sailbot_lat, sailbot_lon):
        self.id = id
        self.lat = sailbot_lat + random.gauss(0, 0.2)
        self.lon = sailbot_lon + random.gauss(0, 0.2)
        self.headingDegrees = random.randint(0, 360)
        self.speedKmph = random.randint(0, 15)

    def move(self):
        # TODO: Update speed calculation latlon <=> kmph
        speed_coeff = 0.001
        
        dx = math.cos(self.headingDegrees)*self.speedKmph*speed_coeff
        dy = math.sin(self.headingDegrees)*self.speedKmph*speed_coeff

        self.lon = self.lon + dx
        self.lat = self.lat + dy

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
        self.ships = []
        for i in range(10):
            self.ships.append(Ship(i, lat, lon))

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
    ais_env = MOCK_AISEnvironment(48.5, -124.8)
    r = rospy.Rate(1) #hz

    while not rospy.is_shutdown():
        ais_env.move_ships()
        data = ais_env.make_ros_message()
        ais_env.publisher.publish(data)
        r.sleep()
