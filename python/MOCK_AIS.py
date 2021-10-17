#!/usr/bin/env python
import rospy
import random
import json

from geopy.distance import distance
from utilities import headingToBearingDegrees, PORT_RENFREW_LATLON, get_rosparam_or_default_if_invalid
from std_msgs.msg import Int32

from sailbot_msg.msg import AISShip, AISMsg, GPS

# Constants
AIS_PUBLISH_PERIOD_SECONDS = 0.1  # Keep below 1.0 for smoother boat motion


def createRandomSimulatedShip(referenceLat, referenceLon, mmsi=None):
    '''Creates a random simulated ship near the reference lat lon.
       Sets a random speed and heading and distance from reference.

    Args:
       referenceLat (float): Latitude that the ship should be near
       referenceLon (float): Longitude that the ship should be near
       mmsi (int/str): mmsi of the ship, if None, then uses random int

    Returns:
       SimulatedShip near the reference latlon
    '''

    def getRandomNearbyLatlon(referenceLatlon, minDistKm, maxDistKm):
        referenceLat, referenceLon = referenceLatlon
        randomDistFromReferenceKm = abs(random.uniform(minDistKm, maxDistKm))
        randomBearingDegrees = random.uniform(0, 360)
        nearbyLatlon = distance(kilometers=randomDistFromReferenceKm).destination(
            point=(referenceLat, referenceLon), bearing=randomBearingDegrees)
        return (nearbyLatlon.latitude, nearbyLatlon.longitude)

    randomLat, randomLon = getRandomNearbyLatlon(referenceLatlon=(
        referenceLat, referenceLon), minDistKm=5, maxDistKm=50)
    randomHeadingDegrees = random.randint(0, 360)
    randomSpeedKmph = random.randint(0, 15)
    if mmsi is None:
        mmsi = random.randint(0, 100000)
    return SimulatedShip(
        MMSI=mmsi,
        lat=randomLat,
        lon=randomLon,
        heading=randomHeadingDegrees,
        speed=randomSpeedKmph)


class Ship:
    '''Base class for storing ship data'''

    def __init__(self, MMSI, lat, lon, heading, speed):
        self.id = MMSI
        self.lat = lat
        self.lon = lon
        self.headingDegrees = heading
        self.speedKmph = speed

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


class SimulatedShip(Ship):
    '''Simulated ship that can be moved over time by the simulation'''

    def move(self, movement_time_seconds):
        distanceTraveledKm = self.speedKmph * movement_time_seconds / 3600
        bearingOfTravelDegrees = headingToBearingDegrees(self.headingDegrees)
        boatLatlon = distance(kilometers=distanceTraveledKm).destination(
            point=(self.lat, self.lon), bearing=bearingOfTravelDegrees)
        self.lat = boatLatlon.latitude
        self.lon = boatLatlon.longitude


class MOCK_AISEnvironment:
    '''Class to keep track of ships surroudning the sailbot'''

    def __init__(self, sailbot_lat, sailbot_lon):
        self.sailbot_lat = sailbot_lat
        self.sailbot_lon = sailbot_lon

        # Setup ros objects
        rospy.init_node('MOCK_AIS', anonymous=True)
        self.publisher = rospy.Publisher("AIS", AISMsg, queue_size=4)
        rospy.Subscriber('/new_boats', AISShip, self.new_boat_callback)
        rospy.Subscriber('/delete_boats', Int32, self.remove_boat_callback)
        rospy.Subscriber('/GPS', GPS, self.gps_callback)

        # Set random seed. Must be called AFTER rospy.init_node and BEFORE
        # making random ships
        self.set_random_seed()

        # Create ships
        self.publishPeriodSeconds = AIS_PUBLISH_PERIOD_SECONDS
        self.ships = []

        # Use stored boats
        ais_file = rospy.get_param('ais_file', default=None)
        if ais_file:
            f = open(ais_file, 'r')
            ship_list = json.load(f)
            for ship in ship_list:
                self.ships.append(SimulatedShip(*ship))
        # Create random new boats
        else:
            numShips = rospy.get_param('num_ais_ships', default=5)
            for i in range(numShips):
                self.ships.append(
                    createRandomSimulatedShip(
                        referenceLat=sailbot_lat,
                        referenceLon=sailbot_lon,
                        mmsi=i))

    def set_random_seed(self):
        randomSeed = get_rosparam_or_default_if_invalid('random_seed', default=None, rosparam_type_cast=str)
        random.seed(randomSeed)

    def move_ships(self):
        speedup = rospy.get_param('speedup', default=1.0)
        movement_time_seconds = speedup * self.publishPeriodSeconds

        for i in (i for i in range(len(self.ships)) if self.ships):
            self.ships[i].move(movement_time_seconds)

            # If simulated ship out of range, remove and add new one
            if isinstance(self.ships[i], SimulatedShip):
                if distance(
                    (self.ships[i].lat,
                     self.ships[i].lon),
                    (self.sailbot_lat,
                     self.sailbot_lon)).km > 60.0:
                    mmsi = self.ships[i].id
                    rospy.loginfo(
                        "MMSI " +
                        str(mmsi) +
                        " went out of bounds, moving it closer to the sailbot")
                    del self.ships[i]
                    self.ships.insert(
                        i,
                        createRandomSimulatedShip(
                            referenceLat=self.sailbot_lat,
                            referenceLon=self.sailbot_lon,
                            mmsi=i))

    def make_ros_message(self):
        return AISMsg([ship.make_ros_message() for ship in self.ships])

    def new_boat_callback(self, msg):
        self.ships.append(
            SimulatedShip(
                msg.ID,
                msg.lat,
                msg.lon,
                msg.headingDegrees,
                msg.speedKmph))

    def remove_boat_callback(self, msg):
        self.ships[:] = [
            ship for ship in self.ships if not ship.id == msg.data]

    def gps_callback(self, msg):
        self.sailbot_lat = msg.lat
        self.sailbot_lon = msg.lon

    def check_new_num_ship(self):
        updated_num_ships = rospy.get_param('num_ais_ships', default=5)
        if len(self.ships) == updated_num_ships:
            return
        elif len(self.ships) > updated_num_ships:
            while len(self.ships) != updated_num_ships:
                self.ships.pop()
        elif len(self.ships) < updated_num_ships:
            for i in range(updated_num_ships - len(self.ships)):
                self.ships.append(
                    createRandomSimulatedShip(
                        referenceLat=self.sailbot_lat,
                        referenceLon=self.sailbot_lon,
                        mmsi=i))
        return


if __name__ == '__main__':
    ais_env = MOCK_AISEnvironment(
        PORT_RENFREW_LATLON.lat,
        PORT_RENFREW_LATLON.lon)
    r = rospy.Rate(1.0 / ais_env.publishPeriodSeconds)  # hz

    while not rospy.is_shutdown():
        data = ais_env.make_ros_message()
        ais_env.check_new_num_ship()
        ais_env.move_ships()

        data = ais_env.make_ros_message()
        ais_env.publisher.publish(data)
        r.sleep()
