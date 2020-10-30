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
        for ship in self.ships:
            dist = distance((ship.lat, ship.lon), (self.lat, self.lon)).km

            if dist < self.collision_radius:
                if ship not in self.collidingBoats:
                    # if ship in self.warningBoats:
                    #     self.warningBoats.remove(ship)
                    #     rospy.logfatal("Warned obstacle has collided with boat")

                    rospy.logwarn("dist: {}. numboats: {}. times_collided: {}. ship ID: {}"
                                  .format(dist, len(self.collidingBoats), self.times_collided, ship.ID))
                    self.times_collided += 1
                    self.collidingBoats.append(ship)
                    rospy.logfatal("Boat has collided with obstacle. Collision radius {}km.\n"
                                   "\tActual distance to boat: {}km. Collision number: {}"
                                   .format(self.collision_radius, dist, self.times_collided))
                    utilities.takeScreenshot()
                    rospy.logwarn("dist: {}. numboats: {}. times_collided: {}. ship ID: {}"
                                  .format(dist, len(self.collidingBoats), self.times_collided, ship.ID))
                    return

            # elif dist < self.warn_radius:
            #     if ship not in self.warningBoats and ship not in self.collidingBoats:
            #         self.times_warned += 1
            #         self.warningBoats.append(ship)
            #         rospy.logwarn("Boat is close to obstacle. Within {}km of boat.\n\tActual distance to boat: {}km. "
            #                       "Warning number: {}" .format(self.warn_radius, dist, self.times_warned))
            #         utilities.takeScreenshot()
            #         return

            else:
                if ship in self.collidingBoats:
                    rospy.logwarn("dist: {}. numboats: {}. times_collided: {}. ship ID: {}"
                                  .format(dist, len(self.collidingBoats), self.times_collided, ship.ID))
                    rospy.logwarn("Obstacle has moved outside warning radius, {} left."
                                  .format(len(self.collidingBoats) - 1))
                    self.collidingBoats.remove(ship)
                    rospy.logwarn("dist: {}. numboats: {}. times_collided: {}. ship ID: {}"
                                  .format(dist, len(self.collidingBoats), self.times_collided, ship.ID))
                    return
                # elif ship in self.warningBoats:  # Warned obstacle number {} has ....
                #     rospy.logwarn("Obstacle has moved outside warning radius, {} left."
                #                   .format(len(self.warningBoats) - 1))  # self.warningBoats[ship], in .format
                #     self.warningBoats.remove(ship)
                #     return

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

    rospy.loginfo("Total number of collisions: {}. Total number of warnings: {}."
                  .format(collision_checker.get_times_collided(), collision_checker.get_times_warned()))
