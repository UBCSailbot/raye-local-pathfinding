#! /usr/bin/env python
import rospy
from local_pathfinding.msg import AISMsg, GPS, path, latlon, windSensor
from geopy.distance import distance
from MOCK_AIS import AIS_PUBLISH_PERIOD_SECONDS

COLLISION_RADIUS = 0.1
WARN_RADIUS = 0.3
CHECK_PERIOD = AIS_PUBLISH_PERIOD_SECONDS 

class CollisionChecker:
    def __init__(self, collision_radius, warn_radius):
        self.collision_radius = collision_radius
        self.warn_radius = warn_radius
        rospy.init_node('collision_checker', anonymous=True)
        rospy.Subscriber('/AIS', AISMsg, self.AIS_callback)
        rospy.Subscriber('/GPS', GPS, self.GPS_callback)
        self.ships = rospy.wait_for_message('/AIS', AISMsg).ships
        self.lat = rospy.wait_for_message('/GPS', GPS).lat
        self.lon = rospy.wait_for_message('/GPS', GPS).lon

    def GPS_callback(self, data):
        self.lat = data.lat
        self.lon = data.lon

    def AIS_callback(self, data):
        self.ships = data.ships

    def check_for_collisions(self):
        for ship in self.ships:
            dist = distance((ship.lat, ship.lon), (self.lat, self.lon)) 
            if dist < self.collision_radius:
                rospy.logfatal("Boat has collided with obstacle")

            elif dist < self.warn_radius:
                rospy.logwarn("Sailbot is within {}km of an obstacle. Close to collision".format(self.warn_radius))

if __name__=='__main__':
    collision_checker = CollisionChecker(COLLISION_RADIUS, WARN_RADIUS)
    rate = rospy.Rate(1 / CHECK_PERIOD)

    while not rospy.is_shutdown():
        collision_checker.check_for_collisions()
        rate.sleep() 
