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

def getXYLimits(xy0, xy1):
    xPLim = max([xy0[0], xy1[0]])
    xNLim = min([xy0[0], xy1[0]])
    yPLim = max([xy0[1], xy1[1]])
    yNLim = min([xy0[1], xy1[1]])

    # Scale to ensure that both dimensions are equal in width
    xWidth = xPLim - xNLim
    yWidth = yPLim - yNLim
    if xWidth > yWidth:
        yPLim = xWidth/yWidth * yPLim
        yNLim = xWidth/yWidth * yNLim
    else:
        xPLim = yWidth/xWidth * xPLim
        xNLim = yWidth/xWidth * xNLim

    # Scale for extra space
    extraWidth = xPLim - xNLim
    xPLim += 0.2*extraWidth
    xNLim -= 0.2*extraWidth
    yPLim += 0.2*extraWidth
    yNLim -= 0.2*extraWidth
    return xPLim, xNLim, yPLim, yNLim

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

        # Convert values from latlon to XY, relative to the nextGlobalWaypoint
        positionXY = latlonToXY(state.position, nextGlobalWaypoint)
        nextGlobalWaypointXY = latlonToXY(nextGlobalWaypoint, nextGlobalWaypoint)
        nextLocalWaypointXY = latlonToXY(nextLocalWaypoint, nextGlobalWaypoint)
        localPathXY = [latlonToXY(localWaypoint, nextGlobalWaypoint) for localWaypoint in localPath]
        shipsXY = [latlonToXY(latlon(ship.lat, ship.lon), nextGlobalWaypoint) for ship in state.AISData.ships]

        # Set xy bounds based on first localWaypoint and nextGlobalWaypoint
        xPLim, xNLim, yPLim, yNLim = getXYLimits(localPathXY[0], nextGlobalWaypointXY)
        plt.xlim([xNLim, xPLim])
        plt.ylim([yNLim, yPLim])

        # Plot local path
        localPathX = [xy[0] for xy in localPathXY]
        localPathY = [xy[1] for xy in localPathXY]
        plt.plot(localPathX, localPathY, marker='.', color='g', markersize=10, linewidth=2)               # Small green dots

        # Plot sailbot, next local waypoint, next global waypoint
        plt.plot(nextGlobalWaypointXY[0], nextGlobalWaypointXY[1], marker='*', color='y', markersize=20)  # Yellow start
        plt.plot(nextLocalWaypointXY[0], nextLocalWaypointXY[1], marker='X', color='g', markersize=20)    # Green X
        plt.plot(positionXY[0], positionXY[1], marker=(3,0,state.heading - 90), color='r', markersize=20) # Blue arrow with correct heading

        # Plot AIS
        ax = plt.gca()
        for ship in shipsXY:
            ax.add_patch(plt.Circle((ship[0], ship[1]), radius=100))                                      # Multiple circles

        # Show wind direction arrow and wind speed text
        arrowLength = min(xPLim - xNLim, yPLim - yNLim) / 8
        arrowCenter = (xNLim + 1.5*arrowLength, yPLim - 1.5*arrowLength)
        windSpeedText = plt.text(arrowCenter[0], arrowCenter[1] + 1.1*arrowLength, "Wind Speed: {}".format(state.windSpeed), ha='center')

        arrowStart = (arrowCenter[0] - 0.5*arrowLength*math.cos(state.windDirection), arrowCenter[1] - 0.5*arrowLength*math.sin(state.windDirection))
        windDirection = plt.arrow(arrowStart[0], arrowStart[1], arrowLength*math.cos(state.windDirection), arrowLength*math.sin(state.windDirection), head_width=10, head_length=10, fc='k', ec='k')
        ax.add_patch(windDirection)

        plt.draw()
        plt.pause(0.001)

        # Sleep then clear plot
        r.sleep()
        plt.clf()
