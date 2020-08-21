#!/usr/bin/env python
import rospy
import math
import utilities as util
from local_pathfinding.msg import AISShip, latlon, addBoat, GPS, path

# Globals for callbacks
localWaypoint = latlon()
gps = latlon()
localPath = []

trailingDistance = 5


def command_callback(msg):
    if msg.addType == "latlon":
        add_pub.publish(msg.ship)
    if msg.addType == "nextWaypoint":
        add_pub.publish(AISShip(msg.ship.ID, localWaypoint.lat, localWaypoint.lon, msg.ship.headingDegrees,
                                msg.ship.speedKmph))
    if msg.addType == "onBoat":
        add_pub.publish(AISShip(msg.ship.ID, gps.lat, gps.lon, msg.ship.headingDegrees, msg.ship.speedKmph))
    if msg.addType == "index":
        if msg.waypointIndex < len(localPath) and msg.waypointIndex >= 0:
            add_pub.publish(AISShip(msg.ship.ID, localPath[msg.waypointIndex].lat, localPath[msg.waypointIndex].lon,
                                    msg.ship.headingDegrees, msg.ship.speedKmph))
            rospy.loginfo("new ship generated at index {} of local path".format(msg.waypointIndex))
        else:
            rospy.loginfo("invalid index passed")
    if msg.addType == "trailing":
        coords = getTrailingBoatLatlon(gps)
        add_pub.publish(AISShip(msg.ship.ID, coords.lat, coords.lon, gps.headingDegrees, gps.speedKmph + 10))



def lwp_callback(coordinates):
    global localWaypoint
    localWaypoint = coordinates


def gps_callback(msg):
    global gps
    # gps = latlon(msg.lat, msg.lon)
    gps = msg


def localPath_callback(msg):
    global localPath
    localPath = msg.waypoints

def getTrailingBoatLatlon(GPS):
    headingRad = math.radians(GPS.headingDegrees)
    m = math.tan(headingRad)
    dx = trailingDistance / (1 + m**2)**0.5
    dy = m * dx

    if math.sin(headingRad) > 0:
        dy = -dy
    if math.cos(headingRad) > 0:
        dx = -dx

    return util.XYToLatlon([dx, dy], latlon(GPS.lat, GPS.lon))
    


if __name__ == "__main__":
    rospy.init_node('addBoatsToPath', anonymous=True)
    rospy.Subscriber('/boat_on_path', addBoat, command_callback)
    rospy.Subscriber('/nextLocalWaypoint', latlon, lwp_callback)
    rospy.Subscriber('/GPS', GPS, gps_callback)
    rospy.Subscriber('/localPath', path, localPath_callback)
    add_pub = rospy.Publisher('/new_boats', AISShip, queue_size=4)
    rospy.Rate(1)
    rospy.spin()
