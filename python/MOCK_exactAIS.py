#!/usr/bin/env python
import rospy
import json
import urllib2
import time
import datetime
import os

from utilities import bearingToHeadingDegrees, PORT_RENFREW_LATLON, XYToLatlon

from MOCK_AIS import Ship
from sailbot_msg.msg import AISMsg, GPS, latlon

# Constants
AIS_PUBLISH_PERIOD_SECONDS = 0.1  # Keep below 1.0 for smoother boat motion


# Inherited from MOCK_AIS.py
class RealShip(Ship):
    '''Real ship from real AIS data'''
    pass


class MOCK_AISEnvironment:
    '''Class to keep track of ships surrounding the sailbot'''
    def __init__(self, sailbot_lat, sailbot_lon):
        self.token = rospy.get_param('AIS_token', default='')
        self.sailbot_lat = sailbot_lat
        self.sailbot_lon = sailbot_lon

        # Setup ros objects
        rospy.init_node('MOCK_exactAIS', anonymous=True)
        self.publisher = rospy.Publisher("AIS", AISMsg, queue_size=4)
        rospy.Subscriber('/GPS', GPS, self.gps_callback)

        # Create ships
        self.last_real_ship_pull = time.time()  # The timestamp for the last time we downloaded new ship positions
        self.publishPeriodSeconds = AIS_PUBLISH_PERIOD_SECONDS

        self.last_real_ship_pull = time.time()  # The timestamp for the last time we downloaded new ship positions
        self.get_real_ships()

    def make_ros_message(self):
        return AISMsg([ship.make_ros_message() for ship in self.real_ships])

    def gps_callback(self, msg):
        self.sailbot_lat = msg.lat
        self.sailbot_lon = msg.lon

    def get_real_ships(self):
        lat = self.sailbot_lat
        lon = self.sailbot_lon
        pos = latlon(lat, lon)

        bot_left = XYToLatlon([-30.0, -30.0], pos)
        top_left = XYToLatlon([-30.0, 30.0], pos)
        top_right = XYToLatlon([30.0, 30.0], pos)
        bot_right = XYToLatlon([30.0, -30.0], pos)

        space = "%20"
        pos_list = ""
        pos_list += str(bot_left.lon) + space + str(bot_left.lat) + space
        pos_list += str(top_left.lon) + space + str(top_left.lat) + space
        pos_list += str(top_right.lon) + space + str(top_right.lat) + space
        pos_list += str(bot_right.lon) + space + str(bot_right.lat) + space
        pos_list += str(bot_left.lon) + space + str(bot_left.lat)

        url = 'https://services.exactearth.com/gws/wms?service=WFS&version=1.1.0&authKey=' + self.token
        url += '&request=GetFeature&typeName=exactAIS:LVI&outputFormat=json&'
        url += 'filter=<Filter%20xmlns:gml="http://www.opengis.net/gml">'
        url += '<Intersects><PropertyName>position</PropertyName>'
        url += '<gml:Polygon%20xmlns:gml="http://www.opengis.net/gml"%20srsName="EPSG:4326">'
        url += '<gml:exterior><gml:LinearRing><gml:posList>' + pos_list
        url += '</gml:posList></gml:LinearRing></gml:exterior></gml:Polygon></Intersects></Filter>'
        request = urllib2.Request(url)
        print("Querying the server")
        data = urllib2.urlopen(request)
        data = data.read()
        if data.startswith('{\n"errors"'):
            print("Queried the server too often, waiting for 2 minutes")
            print(data)
            return
        ships = json.loads(data)
        self.real_ships = []
        self.ais_ships_log = []
        for real_ship in ships['features']:
            # Speed is given in knots
            real_ship = real_ship[u'properties']

            new_ship = RealShip(int(real_ship[u'mmsi']),
                                float(real_ship[u'latitude']),
                                float(real_ship[u'longitude']),
                                bearingToHeadingDegrees(float(real_ship[u'cog'])),
                                float(real_ship[u'sog']) * 0.54)
            self.real_ships.append(new_ship)

            log_ship = {
                'ship_class': {
                    'id': real_ship[u'mmsi'],
                    'lat': real_ship[u'latitude'],
                    'lon': real_ship[u'longitude'],
                    'headingDegrees': bearingToHeadingDegrees(float(real_ship[u'cog'])),
                    'speedKmph': float(real_ship[u'sog']) * 0.54
                },
                'length': real_ship[u'length'],
                'width': real_ship[u'width']
            }
            self.ais_ships_log.append(log_ship)
        self.log_ais_ships()

    def log_ais_ships(self):
        # example: '2021-09-18 01:18:19.500744' -> '2021-09-18_01-18-19'
        timestamp = str(datetime.datetime.now()).split('.')[0].replace(' ', '_').replace(':', '-')

        dir_name = 'exactais_ships_log'
        if not os.path.exists(dir_name):
            os.mkdir(dir_name)

        log_path = '{}/{}.json'.format(dir_name, timestamp)
        with open(log_path, 'w') as f:
            json.dump(self.ais_ships_log, f, indent=2)
        rospy.info('Created AIS ships log at {}'.format(os.path.abspath(log_path)))


if __name__ == '__main__':
    ais_env = MOCK_AISEnvironment(PORT_RENFREW_LATLON.lat, PORT_RENFREW_LATLON.lon)
    r = rospy.Rate(1.0 / ais_env.publishPeriodSeconds)  # hz

    while not rospy.is_shutdown():
        # rospy.loginfo(ais_env.real_ships)
        timestamp = time.time()
        # If it's been more than two minutes since last time we downloaded real ship
        # positions, do it again
        if timestamp - ais_env.last_real_ship_pull > 120:
            ais_env.last_real_ship_pull = timestamp
            ais_env.get_real_ships()

        data = ais_env.make_ros_message()
        ais_env.publisher.publish(data)
        r.sleep()
