#!/usr/bin/env python import rospy
import math
import local_pathfinding.msg as msg
from local_pathfinding.msg import AIS_msg, GPS, path, latlon, wind
from utilities import *
from Sailbot import *
from matplotlib import pyplot as plt
import time

localPath = None
nextLocalWaypoint = None
nextGlobalWaypoint = None

def localPathCallback(data):
    global localPath
    localPath = data.path

def nextLocalWaypointCallback(data):
    global nextLocalWaypoint
    nextLocalWaypoint = data

def nextGlobalWaypointCallback(data):
    global nextGlobalWaypoint
    nextGlobalWaypoint = data

if __name__ == '__main__':
    # Setup ros subscribers
    sailbot = Sailbot(nodeName='local_visualizer')
    rospy.Subscriber("MOCK_local_path", path, localPathCallback)
    rospy.Subscriber("MOCK_next_local_waypoint", latlon, nextLocalWaypointCallback)
    rospy.Subscriber("MOCK_next_global_waypoint", latlon, nextGlobalWaypointCallback)
    r = rospy.Rate(1) #hz

    # Setup plotting
    plt.ion()
    plt.show()

    # Wait for first messages
    while localPath is None or nextLocalWaypoint is None or nextGlobalWaypoint is None:
        rospy.loginfo("Waiting to receive first ROS messages")
        time.sleep(1)

    while not rospy.is_shutdown():
        state = sailbot.getCurrentState()

        positionXY = latlonToXY(state.position, nextGlobalWaypoint)
        nextGlobalWaypointXY = latlonToXY(nextGlobalWaypoint, nextGlobalWaypoint)
        nextLocalWaypointXY = latlonToXY(nextLocalWaypoint, nextGlobalWaypoint)
        localPathXY = [latlonToXY(localWaypoint, nextGlobalWaypoint) for localWaypoint in localPath]
        shipsXY = [latlonToXY(latlon(ship.lat, ship.lon), nextGlobalWaypoint) for ship in state.AISData.ships]

        # Keep same axis xy limits unless change is needed
        extra = 100
        xPLim = max([localPathXY[0][0], nextGlobalWaypointXY[0]]) + extra
        xNLim = min([localPathXY[0][0], nextGlobalWaypointXY[0]]) - extra
        yPLim = max([localPathXY[0][1], nextGlobalWaypointXY[1]]) + extra
        yNLim = min([localPathXY[0][1], nextGlobalWaypointXY[1]]) - extra

        plt.xlim([xNLim, xPLim])
        plt.ylim([yNLim, yPLim])

        # Plot sailbot, next local waypoint, next global waypoint
        plt.plot(positionXY[0], positionXY[1], marker='o', color='b')
        plt.plot(nextGlobalWaypointXY[0], nextGlobalWaypointXY[1], marker='*', color='y', markersize=20)
        plt.plot(nextLocalWaypointXY[0], nextLocalWaypointXY[1], marker='X', color='g', markersize=20)

        # Plot local path
        localPathX = [xy[0] for xy in localPathXY]
        localPathY = [xy[1] for xy in localPathXY]
        plt.plot(localPathX, localPathY, marker='.', color='g', markersize=10, linewidth=2)

        # Plot AIS
        ax = plt.gca()
        for ship in shipsXY:
            ax.add_patch(plt.Circle((ship[0], ship[1]), radius=100))

        # Show wind direction arrow and wind speed text
        arrowLength = min(xPLim - xNLim, yPLim - yNLim) / 8
        arrowCenter = (xNLim + 1.5*arrowLength, yPLim - 1.5*arrowLength)
        arrowStart = (arrowCenter[0] - 0.5*arrowLength*math.cos(state.windDirection), arrowCenter[1] - 0.5*arrowLength*math.sin(state.windDirection))
        # windSpeedAnnotation = plt.annotate("Wind Speed: {}".format(state.windSpeed), (arrowCenter[0], arrowCenter[1] + 1.1*arrowLength), ha='center')
        windSpeedText = plt.text(arrowCenter[0], arrowCenter[1] + 1.1*arrowLength, "Wind Speed: {}".format(state.windSpeed), ha='center')
        windDirection = plt.arrow(arrowStart[0], arrowStart[1], arrowLength*math.cos(state.windDirection), arrowLength*math.sin(state.windDirection), head_width=10, head_length=10, fc='k', ec='k')
        ax.add_patch(windDirection)

        plt.draw()
        plt.pause(0.001)

        # Sleep then clear plot
        r.sleep()
        plt.clf()
