#!/usr/bin/env python
import rospy
import math
import random
import urllib2
import json
import time

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

class RealShip(Ship):
    def __init__(self, MMSI, lat, lon, heading, speed):
        self.id = MMSI
        self.lat = lat
        self.lon = lon
        self.headingDegrees = heading
        self.speedKmph = speed


class MOCK_AISEnvironment: 
    # Just a class to keep track of the ships surrounding the sailbot
    def __init__(self, lat, lon):
        self.use_real_ships = rospy.get_param('use_real_ships', default=False)
        if self.use_real_ships:
            self.last_real_ship_pull = time.time() # The timestamp for the last time we downloaded new ship positions
            self.get_real_ships()
        else:
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

        if self.use_real_ships:
            for ship in self.real_ships:
                ship_list.append(ship.make_ros_message())
        else:
            for i in range(10):
                ship_list.append(self.ships[i].make_ros_message())
       
        return msg.AISMsg(ship_list)

    def get_real_ships(self):
        api_key = "6dc3e9efa43a316d866eb51b3cc0a64922d761ec"
        url = "https://services.marinetraffic.com/api/exportvessels/v:8/" + api_key + "/timespan:10/protocol:json"
        request = urllib2.Request(url)
        # Set the user agent to something so our request is accepted
        request.add_header("User-Agent", "Mozilla/5.0 (Windows NT 6.0; WOW64; rv:24.0) Gecko/20100101 Firefox/24.0")
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
        for real_ship in ships:
            # https://www.marinetraffic.com/no/ais-api-services/documentation/api-service:ps05/_:a7be777368c393257ded7cf842173763#response-PS05
            # Speed is given in knots * 10
            new_ship = RealShip(int(real_ship[0]), float(real_ship[3]), float(real_ship[4]), float(real_ship[6]), float(real_ship[5]) * 0.1 * 0.54)
            self.real_ships.append(new_ship)


if __name__ == '__main__':
    ais_env = MOCK_AISEnvironment(48.5, -124.8)
    r = rospy.Rate(1) #hz

    while not rospy.is_shutdown():
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
