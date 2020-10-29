#! /usr/bin/env python
import rospy
import utilities
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
        self.collidingBoats = list()
        self.warningBoats = list()

    def GPS_callback(self, data):
        self.lat = data.lat
        self.lon = data.lon

    def AIS_callback(self, data):
        self.ships = data.ships

    def check_for_collisions(self):
        currentCollidingBoats = list()
        currentWarningBoats = list()
        pastCollidingBoats = list()
        pastWarningBoats = list()

        for ship in self.ships:
            dist = distance((ship.lat, ship.lon), (self.lat, self.lon)).km

            if dist < self.collision_radius:  # hashmap to keep track of order?
                if ship not in self.collidingBoats:
                    currentCollidingBoats.append(ship)
                    currentWarningBoats.append(ship)
                    self.times_collided += 1
                    rospy.logfatal("Boat has collided with obstacle. Collision radius {}km.\n"
                                   "\tActual distance to boat: {}km. Collision number: {}"
                                   .format(self.collision_radius, dist, self.times_collided))
                    utilities.takeScreenshot()

            elif dist < self.warn_radius:
                if ship not in self.warningBoats:
                    currentWarningBoats.append(ship)
                    self.times_warned += 1
                    rospy.logwarn("Boat is close to obstacle. Within {}km of boat.\n\tActual distance to boat: {}km. "
                                  "Warning number: {}" .format(self.warn_radius, dist, self.times_warned))
                    utilities.takeScreenshot()

            if dist >= self.collision_radius and ship in self.collidingBoats:
                pastCollidingBoats.append(ship)
                rospy.logwarn("An obstacle has moved outside collision radius.")  # get list size

            if dist >= self.warn_radius and ship in self.warningBoats:
                pastWarningBoats.append(ship)
                rospy.logwarn("An obstacle has moved outside warning radius.")

        for ship in pastCollidingBoats:
            self.collidingBoats.remove(ship)

        for ship in pastWarningBoats:
            self.warningBoats.remove(ship)

        self.collidingBoats.extend(currentCollidingBoats)
        self.warningBoats.extend(currentWarningBoats)

    def get_times_collided(self):
        return self.times_collided

    def get_times_warned(self):
        return self.times_warned


if __name__ == '__main__':
    collision_checker = CollisionChecker(COLLISION_RADIUS_KM, WARN_RADIUS_KM)

    rate = rospy.Rate(1 / CHECK_PERIOD)

    while not rospy.is_shutdown():
        collision_checker.check_for_collisions()
        rate.sleep()

    print("Number of collisions: {}. Number of warnings: {}."
          .format(collision_checker.get_times_collided(), collision_checker.get_times_warned()))
