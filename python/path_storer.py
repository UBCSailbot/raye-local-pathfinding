#! /usr/bin/env python
import rospy
import os
import sailbot_msg.msg as msg
from std_msgs.msg import String, Float64
from datetime import datetime
from datetime import date
import pickle
import matplotlib.pyplot as plt
import numpy as np


CHECK_PERIOD_SECONDS = 1.0
PATH_COST_DIFFERENCE_TOLERANCE = 0.1

# Output paths
dateStr = date.today().strftime("%b-%d-%Y")
timeStr = datetime.now().strftime('%H-%M-%S')
ABS_PATH_TO_THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
ABS_PATH_TO_OUTPUT_DIR = os.path.join(ABS_PATH_TO_THIS_FILE_DIR, "../output/{}_{}".format(dateStr, timeStr))
ABS_PATH_TO_OUTPUT_PATH_FILE = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "path.pkl")
ABS_PATH_TO_OUTPUT_PATH_COST_FILE = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "pathCost.pkl")
ABS_PATH_TO_OUTPUT_PATH_COST_BREAKDOWN_FILE = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "pathCostBreakdown.pkl")
ABS_PATH_TO_OUTPUT_COST_PLOT_FILE = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "cost_plot.png")
ABS_PATH_TO_OUTPUT_COST_BREAKDOWN_PLOT_FILE = os.path.join(ABS_PATH_TO_OUTPUT_DIR, "cost_breakdown_plot.png")


class PathStorer:
    def __init__(self):
        rospy.init_node('path_storer', anonymous=True)
        rospy.Subscriber('localPath', msg.path, self.path_callback)
        rospy.Subscriber('localPathCost', Float64, self.pathCost_callback)
        rospy.Subscriber('localPathCostBreakdown', String, self.pathCostBreakdown_callback)

        self.paths = []
        self.pathCosts = []
        self.pathCostBreakdowns = []

        self.currentPath = None
        self.currentPathCost = None
        self.currentPathCostBreakdown = None

    def path_callback(self, data):
        self.currentPath = data.waypoints

    def pathCost_callback(self, data):
        self.currentPathCost = data.data

    def pathCostBreakdown_callback(self, data):
        self.currentPathCostBreakdown = data.data

    def store_path(self):
        if self.currentPath is None or self.currentPathCost is None or self.currentPathCostBreakdown is None:
            rospy.logdebug("Path not received yet.")
            return

        if len(self.pathCosts) == 0 or abs(self.currentPathCost - self.pathCosts[-1]) >= PATH_COST_DIFFERENCE_TOLERANCE:
            rospy.logdebug("New path detected. Storing path.")
            self.paths.append(self.currentPath)
            self.pathCosts.append(self.currentPathCost)
            self.pathCostBreakdowns.append(self.currentPathCostBreakdown)
        else:
            rospy.logdebug("Same path detected. Not storing path.")


if __name__ == '__main__':
    path_storer = PathStorer()
    rate = rospy.Rate(1 / CHECK_PERIOD_SECONDS)

    while not rospy.is_shutdown():
        path_storer.store_path()
        rate.sleep()

    # Save stored path information to pickle files
    if not os.path.exists(ABS_PATH_TO_OUTPUT_DIR):
        os.makedirs(ABS_PATH_TO_OUTPUT_DIR)
    with open(ABS_PATH_TO_OUTPUT_PATH_FILE, 'wb') as handle:
        pickle.dump(path_storer.paths, handle)
    with open(ABS_PATH_TO_OUTPUT_PATH_COST_FILE, 'wb') as handle:
        pickle.dump(path_storer.pathCosts, handle)
    with open(ABS_PATH_TO_OUTPUT_PATH_COST_BREAKDOWN_FILE, 'wb') as handle:
        pickle.dump(path_storer.pathCostBreakdowns, handle)

    # Plot Path cost vs. Time
    rospy.loginfo("About to save cost plot...")
    indices = np.arange(len(path_storer.pathCosts))
    times = CHECK_PERIOD_SECONDS * indices
    plt.figure()
    plt.plot(times, path_storer.pathCosts)
    plt.xlabel("Time elapsed (s)")
    plt.ylabel("Path cost")
    plt.title("Path cost vs. Time")
    plt.savefig(ABS_PATH_TO_OUTPUT_COST_PLOT_FILE)
    rospy.loginfo("Successfully saved cost plot to {}".format(ABS_PATH_TO_OUTPUT_COST_PLOT_FILE))

    # Plot Path cost breakdown vs. Time
    rospy.loginfo("About to save cost breakdown plot...")
    # Get objective names, except for the "Multi" objective, which is the total cost (we want breakdown)
    objectives = [x for x in path_storer.pathCostBreakdowns[0].split(" ") if "Objective" in x and "Multi" not in x]

    # Get the cost from each objective
    costBreakdowns = [[] for _ in objectives]
    for costBreakdownString in path_storer.pathCostBreakdowns:
        costBreakdownStringSplit = costBreakdownString.split(" ")

        # Get weighted cost. Will be in form: [..., 'Weighted', 'cost', '=', '79343.0', ...]
        costBreakdown = [float(costBreakdownStringSplit[i+3]) for i in range(len(costBreakdownStringSplit))
                         if costBreakdownStringSplit[i] == "Weighted"]
        for i in range(len(costBreakdown)):
            costBreakdowns[i].append(costBreakdown[i])

    # Create stacked bar chart. Need to store bottoms to properly stack.
    plt.figure()
    bottoms = np.array([0.0] * len(costBreakdowns[0]))
    barPlots = []
    for costBreakdown in costBreakdowns:
        barPlot = plt.bar(times, costBreakdown, bottom=bottoms, width=CHECK_PERIOD_SECONDS/2)
        barPlots.append(barPlot)
        bottoms += np.array(costBreakdown)

    plt.xlabel("Time elapsed (s)")
    plt.ylabel("Path cost")
    plt.title("Path cost vs. Time")
    plt.legend([bPlot[0] for bPlot in barPlots], objectives)
    plt.savefig(ABS_PATH_TO_OUTPUT_COST_BREAKDOWN_PLOT_FILE)
    rospy.loginfo("Successfully saved cost breakdown plot to {}".format(ABS_PATH_TO_OUTPUT_COST_BREAKDOWN_PLOT_FILE))
