#! /usr/bin/env python
import rospy
import utilities
from sailbot_msg.msg import AISMsg, GPS
from geopy.distance import distance
from MOCK_AIS import AIS_PUBLISH_PERIOD_SECONDS

# match with log_closest_obstacle.py
COLLISION_RADIUS_KM = 0.1
WARN_RADIUS_KM = 0.3
CHECK_PERIOD = AIS_PUBLISH_PERIOD_SECONDS


class CollisionChecker:
    def __init__(self, collision_radius_km=COLLISION_RADIUS_KM, warn_radius_km=WARN_RADIUS_KM, create_ros_node=True):
        if create_ros_node:
            rospy.init_node('collision_checker', anonymous=True)
        self.collision_radius = collision_radius_km
        self.warn_radius = warn_radius_km
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
        self.check_radius(0, self.collision_radius, self.collidingBoats, self.times_collided, "collision")

    def check_for_warnings(self):
        self.check_radius(self.collision_radius, self.warn_radius, self.warningBoats, self.times_warned, "warning")

    # min_radius is inclusive, max_radius is exclusive
    def check_radius(self, min_radius, max_radius, boat_list, counter, message):
        for ship in self.ships:
            dist = distance((ship.lat, ship.lon), (self.lat, self.lon)).km

            if min_radius <= dist < max_radius:
                if ship.ID not in boat_list:
                    counter += 1
                    boat_list.append(ship.ID)
                    rospy.logfatal("Obstacle {} is in {} radius. Actual distance to boat: {}km."
                                   .format(ship.ID, message, dist))
                    utilities.takeScreenshot()

            elif dist >= max_radius:
                if ship.ID in boat_list:
                    boat_list.remove(ship.ID)
                    rospy.logwarn("Obstacle {} has moved outside {} radius, {} left."
                                  .format(ship.ID, message, len(boat_list)))

    def get_times_collided(self):
        return self.times_collided

    def get_times_warned(self):
        return self.times_warned


if __name__ == '__main__':
    collision_checker = CollisionChecker(COLLISION_RADIUS_KM, WARN_RADIUS_KM)
    rate = rospy.Rate(1 / CHECK_PERIOD)

    while not rospy.is_shutdown():
        collision_checker.check_for_collisions()
        collision_checker.check_for_warnings()
        rate.sleep()

    rospy.loginfo("Total number of collisions: {}. Total number of warnings: {}."
                  .format(collision_checker.get_times_collided(), collision_checker.get_times_warned()))
