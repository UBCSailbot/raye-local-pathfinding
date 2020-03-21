#!/usr/bin/env python
import rospy
import json, time
from std_msgs.msg import Int32
from local_pathfinding.msg import AISShip, AISMsg, GPS
global ais_dumped
global gps_dumped
    
def dump_gps(msg):
    global gps_dumped
    dump = json.dumps([msg.lat, msg.lon])
    f = open("gps.json", "w")
    f.write(dump)
    f.close()
    print("GPS recv")
    gps_dumped = True

def dump_ais(msg):
    global ais_dumped
    ship_list = []
    for i in range(len(msg.ships)):
        ship_list.append([
                          msg.ships[i].ID,
                          msg.ships[i].lat,
                          msg.ships[i].lon,
                          msg.ships[i].headingDegrees,
                          msg.ships[i].speedKmph])
    dump = json.dumps(ship_list)
    f = open("ais.json", "w")
    f.write(dump)
    f.close()
    print("AIS recv")
    ais_dumped = True
        
if __name__ == '__main__':
    global ais_dumped, gps_dumped
    ais_dumped = False
    gps_dumped = False
    rospy.init_node('json_dumper', anonymous=True)
    rospy.Subscriber('/GPS', GPS, dump_gps)
    rospy.Subscriber('/AIS', AISMsg, dump_ais)
    while not (ais_dumped and gps_dumped):
        time.sleep(0.5)

