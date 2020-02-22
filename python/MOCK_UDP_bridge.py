#!/usr/bin/env python

import rospy
import math
import random
import socket

import local_pathfinding.msg as msg
from utilities import headingToBearingDegrees, measuredWindToGlobalWind

try:
    import aislib as ais
    import pynmea2 as nmea
except ImportError as err:
    print("ImportError: " + str(err))
    print("")
    print("You seem to be missing a dependency (or two).")
    print("Please run the following commands from within the local-pathfinding directory:")
    print("  pip2 install bitstring")
    print("  pip2 install pynmea2")
    print("")
    exit()



sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

UDP_IP = "127.0.0.1"
UDP_PORT = 65500

# OpenCPN interface
class MOCK_UDPBridge: 
    def __init__(self):
        rospy.init_node('UDPBridge', anonymous=True)
        rospy.Subscriber("GPS", msg.GPS, self.gpsCallback)
        rospy.Subscriber("AIS", msg.AISMsg, self.aisCallback)
        rospy.Subscriber("windSensor", msg.windSensor, self.windCallback)
        rospy.Subscriber("globalPath", msg.path, self.globalPathCallback)
        rospy.Subscriber("localPath", msg.path, self.localPathCallback)
        self.gps = None

    def gpsCallback(self, data):
        rospy.loginfo(data)
        self.gps = data
	lat = str(int(data.lat)*100 + 60*(data.lat - int(data.lat)))
        lon = str(abs(int(data.lon)*100 + 60*(data.lon - int(data.lon)))) # only works on the western hemisphere!
        nmea_msg = nmea.RMC('GP', 'RMC', ('000000', 'A', str(lat), 'N', str(lon), 'W', '2.0', str(headingToBearingDegrees(data.headingDegrees)), '250120', '000.0', 'W'))
        sock.sendto(str(nmea_msg), (UDP_IP, UDP_PORT))

    def aisCallback(self, data):
        rospy.loginfo(data)
        for ship in data.ships:
            aisreport = ais.AISPositionReportMessage(mmsi=ship.ID, lon=int(ship.lon*600000), lat=int(ship.lat*600000), heading=int(headingToBearingDegrees(ship.headingDegrees)) % 360)
            aismsg = ais.AIS(aisreport)
            sock.sendto(aismsg.build_payload(), (UDP_IP, UDP_PORT))

    def windCallback(self, data):
        if not self.gps is None:
            globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(data.measuredSpeedKmph, data.measuredDirectionDegrees, self.gps.speedKmph, self.gps.headingDegrees)
            nmea_msg = nmea.MWV('--', 'MWV', (str(globalWindDirectionDegrees), 'T', str(globalWindSpeedKmph), 'M', 'A'))
            sock.sendto(str(nmea_msg), (UDP_IP, UDP_PORT))
    
    def globalPathCallback(self, data):
        '''
        Send the first 50 global waypoints to OpenCPN
        '''
        msg = "$SAILBOT" + "G" + "50;"
        i = 0
        for wp in data.waypoints:
            msg += str(round(wp.lat, 4)) + "," + str(round(wp.lon, 4)) + ";"
            i += 1
            if i == 50:
                break
        msg += "\n"
        sock.sendto(str(msg), (UDP_IP, UDP_PORT))

    def localPathCallback(self, data):
        msg = "$SAILBOT" + "L" + str(len(data.waypoints)) + ";"
        i = 0
        for wp in data.waypoints:
            msg += str(round(wp.lat, 4)) + "," + str(round(wp.lon, 4)) + ";"
        msg += "\n"
        sock.sendto(str(msg), (UDP_IP, UDP_PORT))



if __name__ == '__main__':
    bridge = MOCK_UDPBridge()
    r = rospy.Rate(1) #hz

    while not rospy.is_shutdown():
        r.sleep()
