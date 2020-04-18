#! /usr/bin/env python
import rospy
from local_pathfinding.msg import AISMsg, GPS, path, latlon, windSensor
from geopy.distance import distance
from MOCK_AIS import AIS_PUBLISH_PERIOD_SECONDS

COLLISION_RADIUS_KM = 0.1
WARN_RADIUS_KM = 0.3
CHECK_PERIOD = AIS_PUBLISH_PERIOD_SECONDS 

class CollisionChecker:
    def __init__(self, collision_radius_km, warn_radius_km):
        self.collision_radius = collision_radius_km
        self.warn_radius = warn_radius_km
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
            dist = distance((ship.lat, ship.lon), (self.lat, self.lon)).km
            if dist < self.collision_radius:
                rospy.logfatal("Boat has collided with obstacle. Collision radius {}km. Actual distance to boat: {}km".format(self.collision_radius, dist))

            elif dist < self.warn_radius:
                rospy.logwarn("Close to collision. Within {}km of boat. Actual distance to boat: {}km".format(self.warn_radius, dist))

if __name__=='__main__':
    collision_checker = CollisionChecker(COLLISION_RADIUS_KM, WARN_RADIUS_KM)
    rate = rospy.Rate(1 / CHECK_PERIOD)

    while not rospy.is_shutdown():
        collision_checker.check_for_collisions()
        rate.sleep() 
