#!/usr/bin/env python
import rospy
from geopy.distance import distance
from local_pathfinding.msg import AISMsg, GPS
# import os
# import local_pathfinding.msg as msg
# from datetime import datetime
# from datetime import date
# import pickle
# import matplotlib.pyplot as plt
# import numpy as np

UPDATE_TIME_SECONDS = 1.0

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

    def find_closest_ship(self):
        closet_ship_dist = None

        for ship in self.ships:
            dist = distance((ship.lat, ship.lon), (self.lat, self.lon)).km

            if closet_ship_dist is None or closet_ship_dist > dist:
                closest_ship_dist = dist

        self.closestDistances.append(closet_ship_dist)


if __name__ == '__main__':
    log_closest_obstacle = LogClosestObstacle()
    rate = rospy.Rate(UPDATE_TIME_SECONDS)

    while not rospy.is_shutdown():
        log_closest_obstacle.find_closest_ship()
        rate.sleep()

    # # Save stored path information to pickle files
    # if not os.path.exists(ABS_PATH_TO_OUTPUT_DIR):
    #     os.makedirs(ABS_PATH_TO_OUTPUT_DIR)
    # with open(ABS_PATH_TO_OUTPUT_PATH_FILE, 'wb') as handle:
    #     pickle.dump(path_storer.paths, handle)
    # with open(ABS_PATH_TO_OUTPUT_PATH_COST_FILE, 'wb') as handle:
    #     pickle.dump(path_storer.pathCosts, handle)
    # with open(ABS_PATH_TO_OUTPUT_PATH_COST_BREAKDOWN_FILE, 'wb') as handle:
    #     pickle.dump(path_storer.pathCostBreakdowns, handle)

    # # Plot Path cost vs. Time
    # rospy.loginfo("About to save cost plot...")
    # indices = np.arange(len(path_storer.pathCosts))
    # times = CHECK_PERIOD_SECONDS * indices
    # plt.figure()
    # plt.plot(times, path_storer.pathCosts)
    # plt.xlabel("Time elapsed (s)")
    # plt.ylabel("Path cost")
    # plt.title("Path cost vs. Time")
    # plt.savefig(ABS_PATH_TO_OUTPUT_COST_PLOT_FILE)
    # rospy.loginfo("Successfully saved cost plot to {}".format(ABS_PATH_TO_OUTPUT_COST_PLOT_FILE))

    # # Plot Path cost breakdown vs. Time
    # rospy.loginfo("About to save cost breakdown plot...")
    # # Get objective names, except for the "Multi" objective, which is the total cost (we want breakdown)
    # objectives = [x for x in path_storer.pathCostBreakdowns[0].split(" ") if "Objective" in x and "Multi" not in x]

    # # Get the cost from each objective
    # costBreakdowns = [[] for _ in objectives]
    # for costBreakdownString in path_storer.pathCostBreakdowns:
    #     costBreakdownStringSplit = costBreakdownString.split(" ")

    #     # Get weighted cost. Will be in form: [..., 'Weighted', 'cost', '=', '79343.0', ...]
    #     costBreakdown = [float(costBreakdownStringSplit[i+3]) for i in range(len(costBreakdownStringSplit))
    #                      if costBreakdownStringSplit[i] == "Weighted"]
    #     for i in range(len(costBreakdown)):
    #         costBreakdowns[i].append(costBreakdown[i])

    # # Create stacked bar chart. Need to store bottoms to properly stack.
    # plt.figure()
    # bottoms = np.array([0.0] * len(costBreakdowns[0]))
    # barPlots = []
    # for costBreakdown in costBreakdowns:
    #     barPlot = plt.bar(times, costBreakdown, bottom=bottoms, width=CHECK_PERIOD_SECONDS/2)
    #     barPlots.append(barPlot)
    #     bottoms += np.array(costBreakdown)

    # plt.xlabel("Time elapsed (s)")
    # plt.ylabel("Path cost")
    # plt.title("Path cost vs. Time")
    # plt.legend([bPlot[0] for bPlot in barPlots], objectives)
    # plt.savefig(ABS_PATH_TO_OUTPUT_COST_BREAKDOWN_PLOT_FILE)
    # rospy.loginfo("Successfully saved cost breakdown plot to {}".format(ABS_PATH_TO_OUTPUT_COST_BREAKDOWN_PLOT_FILE))
