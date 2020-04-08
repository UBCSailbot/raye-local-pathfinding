#!/usr/bin/env python
import rospy
import math
import random
import json

from std_msgs.msg import Float64
from geopy.distance import distance
from utilities import headingToBearingDegrees, PORT_RENFREW_LATLON
from std_msgs.msg import Int32

from local_pathfinding.msg import AISShip, AISMsg, GPS

# Can set random seed to get deterministic start for testing
random.seed(1)

# Constants
AIS_PUBLISH_PERIOD_SECONDS = 0.1  # Keep below 1.0 for smoother boat motion
NUM_AIS_SHIPS = 30

class RandomShip:
    def __init__(self, id, sailbot_lat, sailbot_lon, publishPeriodSeconds):
        self.id = id
        self.headingDegrees = random.randint(0, 360)
        self.speedKmph = random.randint(0, 15)

        # Set AIS boat position to be in about 50km radius around sailbot
        boatLatlon = distance(kilometers=abs(random.randint(5, 50))).destination(point=(sailbot_lat, sailbot_lon), bearing=random.randint(0, 360))
        self.lat = boatLatlon.latitude
        self.lon = boatLatlon.longitude

        self.publishPeriodSeconds = publishPeriodSeconds

    def move(self, speedup):
        # Travel greater distance with speedup
        distanceTraveled = distance(kilometers=self.speedKmph * self.publishPeriodSeconds / 3600 * speedup)
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

    def make_json(self):
        return [
                self.id,
                self.lat,
                self.lon,
                self.headingDegrees,
                self.speedKmph]

class Ship:
    def __init__(self, id, boat_lat, boat_lon, heading, speed, publishPeriodSeconds):
        self.id = id
        self.lat = boat_lat
        self.lon = boat_lon
        self.headingDegrees = heading
        self.speedKmph = speed
        self.publishPeriodSeconds = publishPeriodSeconds

    def move(self, speedup):
        # Travel greater distance with speedup
        distanceTraveled = distance(kilometers=self.speedKmph * self.publishPeriodSeconds / 3600 * speedup)
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

        
class MOCK_AISEnvironment: 
    # Just a class to keep track of the ships surrounding the sailbot
    def __init__(self, lat, lon, ais_file):
        self.publishPeriodSeconds = AIS_PUBLISH_PERIOD_SECONDS
        self.speedup = 1.0
        self.ships = []
        if ais_file:
            f = open(ais_file, 'r')
            ship_list = json.load(f)
            self.numShips = len(ship_list)
            for ship in ship_list:
                self.ships.append(Ship(*ship, publishPeriodSeconds=self.publishPeriodSeconds))
        else:
            self.numShips = NUM_AIS_SHIPS
            for i in range(self.numShips):
                self.ships.append(RandomShip(i, lat, lon, self.publishPeriodSeconds))

        self.sailbot_lat = lat
        self.sailbot_lon = lon

        rospy.init_node('MOCK_AIS', anonymous=True)
        self.publisher = rospy.Publisher("AIS", AISMsg, queue_size=4)
        rospy.Subscriber('/new_boats', AISShip, self.new_boat_callback)
        rospy.Subscriber('/delete_boats', Int32, self.remove_boat_callback)
        rospy.Subscriber('/GPS', GPS, self.gps_callback)
        rospy.Subscriber('speedup', Float64, self.speedup_callback)

    def move_ships(self):
        for i in range(self.numShips):
            self.ships[i].move(self.speedup)
            if isinstance(self.ships[i], RandomShip):
                if distance((self.ships[i].lat, self.ships[i].lon), (self.sailbot_lat, self.sailbot_lon)).km > 60.0:
                    rospy.loginfo("MMSI " + str(self.ships[i].id) + " went out of bounds, moving it closer to the sailbot")
                    del self.ships[i]
                    self.ships.insert(i, RandomShip(i, self.sailbot_lat, self.sailbot_lon, self.publishPeriodSeconds))
      
    def make_ros_message(self):
        rospy.loginfo([ship.id for ship in self.ships])
        ship_list = []
        for i in range(self.numShips):
            ship_list.append(self.ships[i].make_ros_message())
        return AISMsg(ship_list)
    
    def new_boat_callback(self, msg):
        self.ships.append(Ship(msg.ID, msg.lat, msg.lon, msg.headingDegrees, msg.speedKmph, self.publishPeriodSeconds))
        self.numShips += 1
        
    def remove_boat_callback(self, msg):
        self.ships[:] = [ship for ship in self.ships if not ship.id == msg.data] 
        self.numShips = len(self.ships)

    def gps_callback(self, msg):
        self.sailbot_lat = msg.lat
        self.sailbot_lon = msg.lon

    def speedup_callback(self, msg):
        self.speedup = msg.data

if __name__ == '__main__':
    # Get ais_file parameter
    ais_file = rospy.get_param('ais_file', default=None)

    ais_env = MOCK_AISEnvironment(PORT_RENFREW_LATLON.lat, PORT_RENFREW_LATLON.lon, ais_file)
    r = rospy.Rate(1.0 / ais_env.publishPeriodSeconds) #hz

    while not rospy.is_shutdown():
        data = ais_env.make_ros_message()
        ais_env.move_ships()
        ais_env.publisher.publish(data)
        r.sleep()
