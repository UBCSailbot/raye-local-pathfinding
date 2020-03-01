#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from local_pathfinding.msg import AISMsg, AISShip, latlon, addBoat, GPS, path

localWaypoint = latlon()
gps = latlon()
localPath = []

def command_callback(msg):
    if msg.addType == "latlon":
        add_pub.publish(msg.ship)
    if msg.addType == "nextWaypoint":
        add_pub.publish(AISShip(msg.ship.ID, localWaypoint.lat, localWaypoint.lon, msg.ship.headingDegrees, msg.ship.speedKmph))
    if msg.addType == "onBoat":
        add_pub.publish(AISShip(msg.ship.ID, gps.lat, gps.lon, 0, 0))
    if msg.addType == "index":
        if msg.waypointIndex < len(localPath) and msg.waypointIndex >=0:
            add_pub.publish(AISShip(msg.ship.ID, localPath[msg.waypointIndex].lat, localPath[msg.waypointIndex].lon, msg.ship.headingDegrees, msg.ship.speedKmph))
            rospy.loginfo("new ship generated at index {} of local path".format(msg.waypointIndex))
        else:
            rospy.loginfo("invalid index passed")
        
def lwp_callback(coordinates):
    global localWaypoint
    localWaypoint = coordinates 

def gps_callback(msg):
    global gps
    gps = latlon(msg.lat, msg.lon) 

def localPath_callback(msg):
    global localPath
    localPath = msg.waypoints

rospy.init_node('boat_changer', anonymous=True)
rospy.Subscriber('/change_boats', addBoat, command_callback)
rospy.Subscriber('/nextLocalWaypoint', latlon, lwp_callback)
rospy.Subscriber('/GPS', GPS, gps_callback)
rospy.Subscriber('/localPath', path, localPath_callback)
add_pub = rospy.Publisher('/new_boats', AISShip, queue_size=4)
rospy.Rate(1)
rospy.spin()
