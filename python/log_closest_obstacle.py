#!/usr/bin/env python
import rospy
from geopy.distance import distance
from local_pathfinding.msg import AISMsg, GPS
import os
import local_pathfinding.msg as msg
from std_msgs.msg import String, Float64
from datetime import datetime
from datetime import date
import pickle
import matplotlib.pyplot as plt
import numpy as np

UPDATE_TIME_SECONDS = 1.0

# Output paths
dateStr = date.today().strftime("%b-%d-%Y")
timeStr = datetime.now().strftime('%H-%M-%S')
ABS_PATH_TO_THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
ABS_PATH_TO_OUTPUT_DIR = os.path.join(ABS_PATH_TO_THIS_FILE_DIR, "../output/{}_{}".format(dateStr, timeStr))
ABS_PATH_TO_OUTPUT_MIN_DISTANCE_FILE = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "minDistance.pkl")
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

    def find_closest_ship(self):
        closest_ship_dist = None

        for ship in self.ships:
            dist = distance((ship.lat, ship.lon), (self.lat, self.lon)).km

            if closest_ship_dist is None or closest_ship_dist > dist:
                closest_ship_dist = dist

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
    with open(ABS_PATH_TO_OUTPUT_MIN_DISTANCE_FILE, 'wb') as handle:
        pickle.dump(log_closest_obstacle.get_closest_distances(), handle)

    # Plot Path cost vs. Time
    rospy.loginfo("About to save closest distance plot...")
    indices = np.arange(len(log_closest_obstacle.get_closest_distances()))
    times = UPDATE_TIME_SECONDS * indices
    plt.figure()
    plt.plot(times, log_closest_obstacle.get_closest_distances())
    plt.xlabel("Time elapsed (s)")
    plt.ylabel("Closest Distances (km)")
    plt.title("Distance to Closest AIS Boat vs. Time")
    plt.savefig(ABS_PATH_TO_OUTPUT_MIN_DISTANCE_PLOT_FILE)
    rospy.loginfo("Successfully saved closest distance plot to {}".format(ABS_PATH_TO_OUTPUT_MIN_DISTANCE_PLOT_FILE))
