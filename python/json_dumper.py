#!/usr/bin/env python
import rospy
import json, time, datetime
from std_msgs.msg import Int32
from local_pathfinding.msg import AISShip, AISMsg, GPS
global ais_dumped
global gps_dumped
    
def dump_gps(msg):
    global gps_dumped
    now = datetime.datetime.now()
    timestamp = str(now.hour) + str(now.minute) + str(now.second)
    dump = json.dumps([msg.lat, msg.lon, msg.headingDegrees, msg.speedKmph])
    with open("gps-" + timestamp + ".json", "w") as f:
        f.write(dump)
        print("Dumped GPS to gps-" + timestamp + ".json")
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
    now = datetime.datetime.now()
    timestamp = str(now.hour) + str(now.minute) + str(now.second)
    dump = json.dumps(ship_list)
    with open("ais-" + timestamp + ".json", "w") as f:
        f.write(dump)
        print("Dumped AIS to ais-" + timestamp + ".json")
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

