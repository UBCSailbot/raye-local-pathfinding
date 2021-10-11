#!/usr/bin/env python
import rospy
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
boatLat = None
boatLon = None


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
    total_distance_km = distance(init, goal).kilometers
    num_global_waypoints = int(round(total_distance_km / AVG_WAYPOINT_DISTANCE_KM))

    for i in range(1, num_global_waypoints):
        coeff = float(i) / (num_global_waypoints)
        lat = (1 - coeff) * init[0] + coeff * goal[0]
        lon = (1 - coeff) * init[1] + coeff * goal[1]
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

    rospy.init_node('MOCK_global_planner', anonymous=True)
    pub = rospy.Publisher("globalPath", msg.path, queue_size=4)
    r = rospy.Rate(float(1) / PUBLISH_PERIOD_SECONDS)  # Hz

    # Subscribe to GPS to publish new global paths based on boat position
    rospy.Subscriber("GPS", msg.GPS, gpsCallback)

    # Wait to get boat position
    while boatLat is None or boatLon is None:
        rospy.loginfo("Waiting for boat GPS")
        time.sleep(1)
    rospy.loginfo("Received boat GPS")
    init = [boatLat, boatLon]

    # Create goal
    goal_file = rospy.get_param('goal_file', default=None)
    if goal_file:
        with open(goal_file) as f:
            record = json.loads(f.read())
            lat = record[0]
            lon = record[1]
            goal = [lat, lon]
    else:
        rospy.logerr("1")
        goalLat = get_rosparam_or_default_if_invalid('goal_lat', default=MAUI_LATLON.lat)
        rospy.logerr("1")
        goalLon = get_rosparam_or_default_if_invalid('goal_lon', default=MAUI_LATLON.lon)
        rospy.logerr("1")
        goal = [goalLat, goalLon]
        rospy.logerr("1")
        rospy.logerr(goal)

    rospy.logerr("2")
    path = create_path(init, goal)
    rospy.logerr("2")

    # Publish new global path periodically
    # Publish new global path more often with speedup
    republish_counter = 0
    rospy.logerr("2")
    numPublishPeriodsPerUpdate = int(NEW_GLOBAL_PATH_PERIOD_SECONDS / PUBLISH_PERIOD_SECONDS)
    rospy.logerr("2")
    while not rospy.is_shutdown():
        rospy.logerr("3")
        # Send updated global path
        if republish_counter >= numPublishPeriodsPerUpdate:
            republish_counter = 0
            init = [boatLat, boatLon]
            goal = getGoalLatLon(defaultLat=goal[0], defaultLon=goal[1])
            path = create_path(init, goal)
        else:
            speedup = rospy.get_param('speedup', default=1.0)
            republish_counter += speedup

        pub.publish(msg.path(path))
        r.sleep()


if __name__ == '__main__':
    MOCK_global()
