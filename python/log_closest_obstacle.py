#!/usr/bin/env python
import rospy
from geopy.distance import distance
from sailbot_msg.msg import AISMsg, GPS
import os
from datetime import datetime
from datetime import date
import pickle
import matplotlib.pyplot as plt
import numpy as np

UPDATE_TIME_SECONDS = 1.0
PLOT_RANGE = 5

# match with collision_checker.py
COLLISION_RADIUS_KM = 0.1
WARN_RADIUS_KM = 0.3

# Output paths
dateStr = date.today().strftime("%b-%d-%Y")
timeStr = datetime.now().strftime('%H-%M-%S')
ABS_PATH_TO_THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
ABS_PATH_TO_OUTPUT_DIR = os.path.join(ABS_PATH_TO_THIS_FILE_DIR, "../output/{}_{}".format(dateStr, timeStr))
ABS_PATH_TO_OUTPUT_MIN_DISTANCE_FILE = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "minDistance.pkl")
ABS_PATH_TO_OUTPUT_MIN_DISTANCE_INTERACTIVE_FILE = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "minDistanceInteractive.pkl")
ABS_PATH_TO_OUTPUT_MIN_DISTANCE_PLOT_FILE = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "min_distance_plot.png")


class LogClosestObstacle:
    def __init__(self):
        rospy.init_node('log_closest_obstacle', anonymous=True)
        rospy.Subscriber('/AIS', AISMsg, self.AIS_callback)
        rospy.Subscriber('/GPS', GPS, self.GPS_callback)
        self.ships = rospy.wait_for_message('/AIS', AISMsg).ships
        self.lat = rospy.wait_for_message('/GPS', GPS).lat
        self.lon = rospy.wait_for_message('/GPS', GPS).lon

        self.closestDistances = list()

    def get_closest_distances(self):
        return self.closestDistances

    def AIS_callback(self, data):
        self.ships = data.ships

    def GPS_callback(self, data):
        self.lat = data.lat
        self.lon = data.lon

    def get_distance(self, ship):
        return distance((ship.lat, ship.lon), (self.lat, self.lon)).km

    def find_closest_ship(self):
        closest_ship_dist = None if len(self.ships) == 0 else min([self.get_distance(ship) for ship in self.ships])

        if closest_ship_dist is None or len(self.closestDistances) == 0:
            self.closestDistances.append(0)
        else:
            self.closestDistances.append(closest_ship_dist)


if __name__ == '__main__':
    log_closest_obstacle = LogClosestObstacle()
    rate = rospy.Rate(UPDATE_TIME_SECONDS)

    while not rospy.is_shutdown():
        log_closest_obstacle.find_closest_ship()
        rate.sleep()

    # Save stored path information to pickle files
    if not os.path.exists(ABS_PATH_TO_OUTPUT_DIR):
        os.makedirs(ABS_PATH_TO_OUTPUT_DIR)

    # Plot Path cost vs. Time
    rospy.loginfo("About to save closest distance plot...")
    indices = np.arange(len(log_closest_obstacle.get_closest_distances()))
    times = UPDATE_TIME_SECONDS * indices
    warn_radius_line = list()
    [warn_radius_line.append(WARN_RADIUS_KM) for _ in times]
    collision_radius_line = list()
    [collision_radius_line.append(COLLISION_RADIUS_KM) for _ in times]

    graph = plt.figure()
    plt.ylim(0, PLOT_RANGE)
    plt.plot(times, log_closest_obstacle.get_closest_distances())
    plt.plot(times, warn_radius_line, color='yellow')
    plt.plot(times, collision_radius_line, color='red')

    plt.xlabel("Time elapsed (s)")
    plt.ylabel("Closest Distances (km)")
    plt.title("Distance to Closest AIS Boat vs. Time")
    plt.savefig(ABS_PATH_TO_OUTPUT_MIN_DISTANCE_PLOT_FILE)
    with open(ABS_PATH_TO_OUTPUT_MIN_DISTANCE_INTERACTIVE_FILE, 'wb') as handle:
        pickle.dump(graph, handle)
    rospy.loginfo("Successfully saved closest distance plot to {}".format(ABS_PATH_TO_OUTPUT_MIN_DISTANCE_PLOT_FILE))
