#!/usr/bin/env python
import rospy
import math
import random
from geopy.distance import distance
from utilities import headingToBearingDegrees, PORT_RENFREW_LATLON
from std_msgs.msg import Int32

from local_pathfinding.msg import AISShip, AISMsg

class RandomShip:
    def __init__(self, id, sailbot_lat, sailbot_lon, publishPeriodSeconds):
        self.id = id
        self.headingDegrees = random.randint(0, 360)
        self.speedKmph = random.randint(0, 15)

        # Set AIS boat position to be in about 50km radius around sailbot
        boatLatlon = distance(kilometers=abs(random.gauss(10, 5))).destination(point=(sailbot_lat, sailbot_lon), bearing=random.randint(0, 360))
        self.lat = boatLatlon.latitude
        self.lon = boatLatlon.longitude

        self.publishPeriodSeconds = publishPeriodSeconds

    def move(self):
        distanceTraveled = distance(kilometers=self.speedKmph * self.publishPeriodSeconds / 3600)
        boatLatlon = distanceTraveled.destination(point=(self.lat, self.lon), bearing=headingToBearingDegrees(self.headingDegrees))

        self.lon = boatLatlon.longitude
        self.lat = boatLatlon.latitude

    def make_ros_message(self):
        return AISShip(
                    self.id,
                    self.lat,
                    self.lon,
                    self.headingDegrees,
                    self.speedKmph)

class Ship:
    def __init__(self, id, boat_lat, boat_lon, heading, speed):
        self.id = id
        self.lat = boat_lat
        self.lon = boat_lon
        self.headingDegrees = heading
        self.speedKmph = speed

    def make_ros_message(self):
        return AISShip(
                    self.id,
                    self.lat,
                    self.lon,
                    self.headingDegrees,
                    self.speedKmph)

        
class MOCK_AISEnvironment: 
    # Just a class to keep track of the ships surrounding the sailbot
    def __init__(self, lat, lon):
        self.publishPeriodSeconds = 1.0
        self.numShips = 10
        self.ships = []
        for i in range(self.numShips):
            self.ships.append(RandomShip(i, lat, lon, self.publishPeriodSeconds))

        rospy.init_node('MOCK_AIS', anonymous=True)
        self.publisher = rospy.Publisher("AIS", AISMsg, queue_size=4)
        rospy.Subscriber('/new_boats', AISShip, self.new_boat_callback)
        rospy.Subscriber('/delete_boats', Int32, self.remove_boat_callback)

    def move_ships(self):
        for i in range(self.numShips):
            if isinstance(self.ships[i], RandomShip):
                self.ships[i].move()

    def make_ros_message(self):
        rospy.loginfo([ship.id for ship in self.ships])
        ship_list = []
        for i in range(self.numShips):
            ship_list.append(self.ships[i].make_ros_message())
        return AISMsg(ship_list)
    
    def new_boat_callback(self, msg):
        self.ships.append(Ship(msg.ID, msg.lat, msg.lon, msg.headingDegrees, msg.speedKmph))
        self.numShips += 1
        print("in callback")
        
    def remove_boat_callback(self, msg):
        self.ships[:] = [ship for ship in self.ships if not ship.id == msg.data] 
        self.numShips = len(self.ships)

if __name__ == '__main__':
    ais_env = MOCK_AISEnvironment(PORT_RENFREW_LATLON.lat, PORT_RENFREW_LATLON.lon)
    r = rospy.Rate(1.0 / ais_env.publishPeriodSeconds) #hz

    while not rospy.is_shutdown():
        data = ais_env.make_ros_message()
        ais_env.move_ships()
        ais_env.publisher.publish(data)
        r.sleep()
