#!/usr/bin/env python
import rospy
import random
import json
import urllib2
import time

from geopy.distance import distance
from utilities import headingToBearingDegrees, PORT_RENFREW_LATLON
from std_msgs.msg import Int32

from sailbot_msg.msg import AISShip, AISMsg, GPS

# Constants
AIS_PUBLISH_PERIOD_SECONDS = 0.1  # Keep below 1.0 for smoother boat motion
NUM_AIS_SHIPS = rospy.get_param('num_ais_ships', default=5)


class RandomShip:
    def __init__(self, id, sailbot_lat, sailbot_lon, publishPeriodSeconds):
        self.id = id
        self.headingDegrees = random.randint(0, 360)
        self.speedKmph = random.randint(0, 15)

        # Set AIS boat position to be in about 50km radius around sailbot
        distanceFromSailbotKm = distance(kilometers=abs(random.randint(5, 50)))
        boatLatlon = distanceFromSailbotKm.destination(point=(sailbot_lat, sailbot_lon),
                                                       bearing=random.randint(0, 360))
        self.lat = boatLatlon.latitude
        self.lon = boatLatlon.longitude

        self.publishPeriodSeconds = publishPeriodSeconds

    def move(self, speedup):
        # Travel greater distance with speedup
        distanceTraveledKm = distance(kilometers=self.speedKmph * self.publishPeriodSeconds / 3600 * speedup)
        boatLatlon = distanceTraveledKm.destination(point=(self.lat, self.lon),
                                                    bearing=headingToBearingDegrees(self.headingDegrees))

        self.lon = boatLatlon.longitude
        self.lat = boatLatlon.latitude

    def make_ros_message(self):
        return AISShip(self.id, self.lat, self.lon, self.headingDegrees, self.speedKmph)

    def make_json(self):
        return [self.id, self.lat, self.lon, self.headingDegrees, self.speedKmph]


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
        distanceTraveledKm = distance(kilometers=self.speedKmph * self.publishPeriodSeconds / 3600 * speedup)
        boatLatlon = distanceTraveledKm.destination(point=(self.lat, self.lon),
                                                    bearing=headingToBearingDegrees(self.headingDegrees))

        self.lon = boatLatlon.longitude
        self.lat = boatLatlon.latitude

    def make_ros_message(self):
        return AISShip(
            self.id,
            self.lat,
            self.lon,
            (90.0 -self.headingDegrees) % 360,
            self.speedKmph)

class RealShip(Ship):
    def __init__(self, MMSI, lat, lon, heading, speed):
        self.id = MMSI
        self.lat = lat
        self.lon = lon
        self.headingDegrees = heading
        self.speedKmph = speed


class MOCK_AISEnvironment:
    # Just a class to keep track of the ships surrounding the sailbot
    def __init__(self, lat, lon, ais_file):
        self.use_real_ships = rospy.get_param('use_real_ships', default=False)
        self.token = rospy.get_param('AIS_token', default='')
        self.sailbot_lat = lat
        self.sailbot_lon = lon

        # Setup ros objects
        rospy.init_node('MOCK_AIS', anonymous=True)
        self.publisher = rospy.Publisher("AIS", AISMsg, queue_size=4)
        rospy.Subscriber('/new_boats', AISShip, self.new_boat_callback)
        rospy.Subscriber('/delete_boats', Int32, self.remove_boat_callback)
        rospy.Subscriber('/GPS', GPS, self.gps_callback)

        # Set random seed. Must be called AFTER rospy.init_node and BEFORE making random ships
        self.set_random_seed()

        # Create ships
        self.last_real_ship_pull = time.time() # The timestamp for the last time we downloaded new ship positions
        self.publishPeriodSeconds = AIS_PUBLISH_PERIOD_SECONDS
        self.ships = []

        if ais_file:
            f = open(ais_file, 'r')
            ship_list = json.load(f)
            self.numShips = len(ship_list)
            for ship in ship_list:
                self.ships.append(Ship(*ship, publishPeriodSeconds=self.publishPeriodSeconds))
        elif self.use_real_ships:
            self.last_real_ship_pull = time.time() # The timestamp for the last time we downloaded new ship positions
            self.get_real_ships()
        else:
            self.numShips = NUM_AIS_SHIPS
            for i in range(self.numShips):
                self.ships.append(RandomShip(i, lat, lon, self.publishPeriodSeconds))


    def set_random_seed(self):
        randomSeed = rospy.get_param('random_seed', "")
        try:
            randomSeed = int(randomSeed)
            random.seed(randomSeed)
            rospy.loginfo("randomSeed = {}. Setting seed".format(randomSeed))
        except ValueError:
            rospy.loginfo("randomSeed = {}. Not setting seed".format(randomSeed))

    def move_ships(self):
        speedup = rospy.get_param('speedup', default=1.0)
        for i in range(self.numShips):
            self.ships[i].move(speedup)
            if isinstance(self.ships[i], RandomShip):
                if distance((self.ships[i].lat, self.ships[i].lon), (self.sailbot_lat, self.sailbot_lon)).km > 60.0:
                    rospy.loginfo("MMSI " + str(self.ships[i].id) +
                                  " went out of bounds, moving it closer to the sailbot")
                    del self.ships[i]
                    self.ships.insert(i, RandomShip(i, self.sailbot_lat, self.sailbot_lon, self.publishPeriodSeconds))

    def make_ros_message(self):
        ship_list = []
        if self.use_real_ships:
            for ship in self.real_ships:
                ship_list.append(ship.make_ros_message())
        else:
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

    def get_real_ships(self):
        lat = self.sailbot_lat
        lon = self.sailbot_lon
        space = "%20"
        pos_list = ""
        pos_list += str(lon-0.2) + space + str(lat-0.2) + space # bot left
        pos_list += str(lon-0.2) + space + str(lat+0.2) + space # top left
        pos_list += str(lon+0.2) + space + str(lat+0.2) + space # top right
        pos_list += str(lon+0.2) + space + str(lat-0.2) + space # bot right
        pos_list += str(lon-0.2) + space + str(lat-0.2)         # bot left


        url = 'https://services.exactearth.com/gws/wms?service=WFS&version=1.1.0&authKey=' + self.token
        url += '&request=GetFeature&typeName=exactAIS:LVI&outputFormat=json&filter=<Filter%20xmlns:gml="http://www.opengis.net/gml">'
        url += '<Intersects><PropertyName>position</PropertyName><gml:Polygon%20xmlns:gml="http://www.opengis.net/gml"%20srsName="EPSG:4326">'
        url += '<gml:exterior><gml:LinearRing><gml:posList>' + pos_list 
        url += '</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></Intersects></Filter>'
        request = urllib2.Request(url)
        # Set the user agent to something so our request is accepted
        #request.add_header("User-Agent", "Mozilla/5.0 (Windows NT 6.0; WOW64; rv:24.0) Gecko/20100101 Firefox/24.0")
        #request.add_header("User-Agent", "Mozilla/5.0 (X11; Linux x86_64; rv:72.0) Gecko/20100101 Firefox/72.0")
        #request.add_header("User-Agent", "Mozilla/5.0 (Linux; Android 7.0; SM-G892A Build/NRD90M; wv) AppleWebKit/537.36 (KHTML, like Gecko) Version/4.0 Chrome/60.0.3112.107 Mobile Safari/537.36")
        print("Querying the server")
        data = urllib2.urlopen(request)
        data = data.read()
        if data.startswith('{\n"errors"'):
            print("Queried the server too often, waiting for 2 minutes")
            print(data)
            return
        ships = json.loads(data)
        self.real_ships = []
        for real_ship in ships['features']:
            # Speed is given in knots
            real_ship = real_ship[u'properties']
            new_ship = RealShip(int(real_ship[u'mmsi']),
                                float(real_ship[u'latitude']),
                                float(real_ship[u'longitude']),
                                float(real_ship[u'cog']),
                                float(real_ship[u'sog']) * 0.54)
            self.real_ships.append(new_ship)


if __name__ == '__main__':
    # Get ais_file parameter
    ais_file = rospy.get_param('ais_file', default=None)

    ais_env = MOCK_AISEnvironment(PORT_RENFREW_LATLON.lat, PORT_RENFREW_LATLON.lon, ais_file)
    r = rospy.Rate(1.0 / ais_env.publishPeriodSeconds)  # hz

    while not rospy.is_shutdown():
        #rospy.loginfo(ais_env.real_ships)
        if ais_env.use_real_ships:
            timestamp = time.time()
            # If it's been more than two minutes since last time we downloaded real ship
            # positions, do it again
            if timestamp - ais_env.last_real_ship_pull > 120:
                ais_env.last_real_ship_pull = timestamp
                ais_env.get_real_ships()
        else:
            ais_env.move_ships()

        data = ais_env.make_ros_message()
        ais_env.publisher.publish(data)
        r.sleep()
