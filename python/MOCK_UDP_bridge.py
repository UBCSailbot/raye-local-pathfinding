#!/usr/bin/env python

import rospy
import math
import random
import socket

import local_pathfinding.msg as msg

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
        rospy.Subscriber("MOCK_GPS", msg.GPS, self.gpsCallback)
        rospy.Subscriber("MOCK_AIS", msg.AIS_msg, self.aisCallback)
        rospy.Subscriber("MOCK_wind", msg.wind, self.windCallback)
        rospy.Subscriber("MOCK_global_path", msg.path, self.globalPathCallback)
        rospy.Subscriber("MOCK_local_path", msg.path, self.localPathCallback)

    def gpsCallback(self, data):
        # TODO: fix heading
        rospy.loginfo(data)
        nmea_msg = nmea.RMC('GP', 'RMC', ('000000', 'A', str(data.lat*100)[0:7], 'N', str(abs(data.lon)*100)[0:7], 'W', '2.0', str(-math.degrees(data.heading) + 90), '250120', '000.0', 'W'))
        sock.sendto(str(nmea_msg), (UDP_IP, UDP_PORT))

    def aisCallback(self, data):
	# TODO: fix heading
        rospy.loginfo(data)
        for ship in data.ships:
            aisreport = ais.AISPositionReportMessage(mmsi=ship.ID, lon=int(ship.lon*600000), lat=int(ship.lat*600000), heading=int(-math.degrees(ship.heading) + 90) % 360)
            aismsg = ais.AIS(aisreport)
            sock.sendto(aismsg.build_payload(), (UDP_IP, UDP_PORT))

    def windCallback(self, data):
        nmea_msg = nmea.MWV('--', 'MWV', (str(math.degrees(data.direction)), 'T', str(data.speed), 'M', 'A'))
        sock.sendto(str(nmea_msg), (UDP_IP, UDP_PORT))
    
    def globalPathCallback(self, data):
        i = 0
        for wp in data.path:
            if (i % 25 == 0):
                nmea_msg = nmea.WPL('GP', 'WPL', (str(100*wp.lat), 'N', str(100*wp.lon), 'E', 'G' + str(i)))
                sock.sendto(str(nmea_msg), (UDP_IP, UDP_PORT))
            i += 1

    def localPathCallback(self, data):
        i = 0
        for wp in data.path:
            nmea_msg = nmea.WPL('GP', 'WPL', (str(100*wp.lat), 'N', str(100*wp.lon), 'E', 'G' + str(i)))
            sock.sendto(str(nmea_msg), (UDP_IP, UDP_PORT))
            i += 1



if __name__ == '__main__':
    bridge = MOCK_UDPBridge()
    r = rospy.Rate(1) #hz

    while not rospy.is_shutdown():
        r.sleep()
