#!/usr/bin/env python
import rospy
import sys
import json
import time
from utilities import MAUI_LATLON, get_rosparam_or_default_if_invalid

import sailbot_msg.msg as msg
from geopy.distance import distance

# Constants for this class
PUBLISH_PERIOD_SECONDS = 10.0  # Can keep high to simulate real global pathfinding
NEW_GLOBAL_PATH_PERIOD_SECONDS = 10.0
AVG_WAYPOINT_DISTANCE_KM = 30  # TODO: Set this to match global pathfinding

# Global variables for tracking boat position
boatLatlon = None


def gpsCallback(data):
    global boatLatlon
    boatLatlon = msg.latlon(lat=data.lat, lon=data.lon)


def create_path(init, goal):
    path = []

    # Insert the initial position
    path.append(init)

    # Just do some linear interpolation
    total_distance_km = distance((init.lat, init.lon), (goal.lat, goal.lon)).kilometers
    num_global_waypoints = int(round(total_distance_km / AVG_WAYPOINT_DISTANCE_KM))

    for i in range(1, num_global_waypoints):
        coeff = float(i) / (num_global_waypoints)
        lat = (1 - coeff) * init.lat + coeff * goal.lat
        lon = (1 - coeff) * init.lon + coeff * goal.lon
        wp = msg.latlon(lat=lat, lon=lon)
        path.append(wp)

    # Insert the goal
    path.append(goal)
    return path


def getGoalLatlon(defaultLat, defaultLon):
    goalLat = get_rosparam_or_default_if_invalid('goal_lat', default=defaultLat)
    goalLon = get_rosparam_or_default_if_invalid('goal_lon', default=defaultLon)
    return msg.latlon(lat=goalLat, lon=goalLon)


def MOCK_global():
    global boatLatlon

    rospy.init_node('MOCK_global_planner', anonymous=True)
    pub = rospy.Publisher("globalPath", msg.path, queue_size=4)
    r = rospy.Rate(float(1) / PUBLISH_PERIOD_SECONDS)  # Hz

    # Subscribe to GPS to publish new global paths based on boat position
    rospy.Subscriber("GPS", msg.GPS, gpsCallback)

    # Wait to get boat position
    while boatLatlon is None:
        if rospy.is_shutdown():
            rospy.loginfo("rospy.is_shutdown() is True. Exiting")
            sys.exit()
        rospy.loginfo("Waiting for boat GPS")
        time.sleep(1)
    rospy.loginfo("Received boat GPS")
    init = msg.latlon(lat=boatLatlon.lat, lon=boatLatlon.lon)

    # Create goal
    goal_file = rospy.get_param('goal_file', default=None)
    if goal_file:
        with open(goal_file) as f:
            record = json.loads(f.read())
            lat = record[0]
            lon = record[1]
            goal = msg.latlon(lat=lat, lon=lon)
    else:
        goal = getGoalLatlon(defaultLat=MAUI_LATLON.lat, defaultLon=MAUI_LATLON.lon)

    path = create_path(init, goal)

    # Publish new global path periodically
    # Publish new global path more often with speedup
    republish_counter = 0
    numPublishPeriodsPerUpdate = int(NEW_GLOBAL_PATH_PERIOD_SECONDS / PUBLISH_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        # Send updated global path
        if republish_counter >= numPublishPeriodsPerUpdate:
            republish_counter = 0
            init = msg.latlon(lat=boatLatlon.lat, lon=boatLatlon.lon)
            goal = getGoalLatlon(defaultLat=goal.lat, defaultLon=goal.lon)
            path = create_path(init, goal)
        else:
            speedup = rospy.get_param('speedup', default=1.0)
            republish_counter += speedup

        pub.publish(msg.path(path))
        r.sleep()


if __name__ == '__main__':
    MOCK_global()
