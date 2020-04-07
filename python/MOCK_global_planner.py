#!/usr/bin/env python
import rospy
import math
import json
import time
from utilities import PORT_RENFREW_LATLON, MAUI_LATLON

from std_msgs.msg import Float64
import local_pathfinding.msg as msg
from geopy.distance import distance

# Constants for this class
PUBLISH_PERIOD_SECONDS = 10.0  # Can keep high to simulate real global pathfinding
NEW_GLOBAL_PATH_PERIOD_SECONDS = 3600.0 * 24
AVG_WAYPOINT_DISTANCE_KM = 30  # TODO: Set this to match global pathfinding

# Global variables for tracking boat position
boatLat = None
boatLon = None
def gpsCallback(data):
    global boatLat
    global boatLon
    boatLat = data.lat
    boatLon = data.lon

# Global variable for speedup
speedup = 1.0
def speedupCallback(data):
    global speedup
    speedup = data.data

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
    global speedup

    rospy.init_node('MOCK_global_planner', anonymous=True)
    pub = rospy.Publisher("globalPath", msg.path, queue_size=4)
    r = rospy.Rate(float(1) / PUBLISH_PERIOD_SECONDS)  # Hz

    # Subscribe to GPS to publish new global paths based on boat position
    rospy.Subscriber("GPS", msg.GPS, gpsCallback)

    # Subscribe to speedup
    rospy.Subscriber("speedup", Float64, speedupCallback)

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
        goal = [MAUI_LATLON.lat, MAUI_LATLON.lon]

    path = create_path(init, goal)

    # Publish new global path periodically
    # Publish new global path more often with speedup
    republish_counter = 0
    newGlobalPathPeriodSecondsSpeedup = NEW_GLOBAL_PATH_PERIOD_SECONDS / speedup
    numPublishPeriodsPerUpdate = int(NEW_GLOBAL_PATH_PERIOD_SECONDS / PUBLISH_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        # Send updated global path
        if republish_counter >= numPublishPeriodsPerUpdate:
            republish_counter = 0
            init = [boatLat, boatLon]
            path = create_path(init, goal)
        else:
            republish_counter += 1

        pub.publish(msg.path(path))
        r.sleep()

if __name__ == '__main__':
    MOCK_global()
