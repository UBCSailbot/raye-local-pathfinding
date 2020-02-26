#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from local_pathfinding.msg import AISMsg, AISShip, latlon, addBoat, GPS

localWaypoint = latlon()
gps = latlon()

def command_callback(msg):
    if msg.addType == "latlon":
        add_pub.publish(msg.ship)
    if msg.addType == "nextWaypoint":
        add_pub.publish(AISShip(msg.ship.ID, localWaypoint.lat, localWaypoint.lon, 0, 0))
    if msg.addType == "onBoat":
        add_pub.publish(AISShip(msg.ship.ID, gps.lat, gps.lon, 0, 0))
        
def lwp_callback(coordinates):
    global localWaypoint
    localWaypoint = coordinates 

def gps_callback(msg):
    global gps
    gps = latlon(msg.lat, msg.lon) 

rospy.init_node('boat_changer', anonymous=True)
rospy.Subscriber('/change_boats', addBoat, command_callback)
rospy.Subscriber('/nextLocalWaypoint', latlon, lwp_callback)
rospy.Subscriber('/GPS', GPS, gps_callback)
add_pub = rospy.Publisher('/new_boats', AISShip, queue_size=4)
del_pub = rospy.Publisher('delete_boats', Int32, queue_size=4)
rospy.Rate(1)
rospy.spin()
