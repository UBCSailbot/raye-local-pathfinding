#! /usr/bin/env python
import rospy
from local_pathfinding.msg import AISMsg, GPS
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
        self.times_collided = 0
        self.times_warned = 0

    def GPS_callback(self, data):
        self.lat = data.lat
        self.lon = data.lon

    def AIS_callback(self, data):
        self.ships = data.ships

    def check_for_collisions(self, colliding_ship):
        check_ship = colliding_ship

        if check_ship is None:
            for ship in self.ships:
                dist = distance((ship.lat, ship.lon), (self.lat, self.lon)).km

                if dist < self.collision_radius:
                    check_ship = ship
                    self.times_collided = self.times_collided + 1
                    rospy.logfatal("Boat has collided with obstacle. Collision radius {}km.\n"
                                   "\tActual distance to boat: {}km. Collision number: {}"
                                   .format(self.collision_radius, dist, self.times_collided))

                elif dist < self.warn_radius:
                    check_ship = ship
                    self.times_warned = self.times_warned + 1
                    rospy.logwarn("Close to collision. Within {}km of boat.\n\tActual distance to boat: {}km. "
                                  "Warning number: {}" .format(self.warn_radius, dist, self.times_warned))
        else:
            dist = distance((check_ship.lat, check_ship.lon), (self.lat, self.lon)).km
            if not (dist < self.collision_radius or dist < self.warn_radius):
                check_ship = None
                rospy.logwarn("Other ships are not within the collision or warning radius anymore.")

        return check_ship

    def get_times_collided(self):
        return self.times_collided

    def get_times_warned(self):
        return self.times_warned


if __name__ == '__main__':
    collision_checker = CollisionChecker(COLLISION_RADIUS_KM, WARN_RADIUS_KM)

    rate = rospy.Rate(1 / CHECK_PERIOD)

    shipColliding = None
    while not rospy.is_shutdown():
        shipColliding = collision_checker.check_for_collisions(shipColliding)
        rate.sleep()

    print("Number of collisions: {}. Number of warnings: {}."
          .format(collision_checker.get_times_collided(), collision_checker.get_times_warned()))
