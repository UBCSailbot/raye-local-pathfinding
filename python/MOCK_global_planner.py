#!/usr/bin/env python
import rospy
import math

import local_pathfinding.msg as msg
from geopy.distance import distance

# Global variables for tracking boat position
boatLat = 48.5  # Assume start at Port Renfrew (ish)
boatLon = -124.0
def gpsCallback(data):
    global boatLat
    global boatLon
    boatLat = data.lat
    boatLon = data.lon

def create_path(init, goal):
    path = []

    # Insert the initial position
    init_wp = msg.latlon()
    init_wp.lat = init[0]
    init_wp.lon = init[1]
    path.append(init_wp)

    # Just do some linear interpolation
    avg_waypoint_distance_km = 30  # TODO: Set this to match global pathfinding
    total_distance_km = distance(init, goal).kilometers
    num_global_waypoints = int(round(total_distance_km / avg_waypoint_distance_km))

    for i in range(1, num_global_waypoints):
        coeff = float(i)/(num_global_waypoints)
        lat = (1 - coeff)*init[0] + coeff*goal[0]
        lon = (1 - coeff)*init[1] + coeff*goal[1]
        print(lat, lon)
        wp = msg.latlon()
        wp.lat = lat
        wp.lon = lon
        path.append(wp)

    # Insert the goal
    last_wp = msg.latlon()
    last_wp.lat = goal[0]
    last_wp.lon = goal[1]
    path.append(last_wp)
    return path

def MOCK_global():
    global boatLat
    global boatLon
    init = [boatLat, boatLon]
    goal = [20.0, -156.0] # Maui (ish)
    path = create_path(init, goal)

    rospy.init_node('MOCK_global_planner', anonymous=True)
    pub = rospy.Publisher("globalPath", msg.path, queue_size=4)
    publish_period_seconds = 10 # Seconds. TODO: set this rate
    r = rospy.Rate(float(1) / publish_period_seconds)  # Hz

    # Subscribe to GPS to publish new global paths based on boat position
    rospy.Subscriber("GPS", msg.GPS, gpsCallback)

    republish_counter = 0
    while not rospy.is_shutdown():
        # Send updated global path every X periods
        if republish_counter >= 100:
            republish_counter = 0
            init = [boatLat, boatLon]
            path = create_path(init, goal)
        else:
            republish_counter += 1

        pub.publish(msg.path(path))
        r.sleep()

if __name__ == '__main__':
    MOCK_global()
