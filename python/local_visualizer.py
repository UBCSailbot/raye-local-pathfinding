#!/usr/bin/env python import rospy
import math
import local_pathfinding.msg as msg
from local_pathfinding.msg import AIS_msg, GPS, path, latlon, wind
from utilities import *
from Sailbot import *
from matplotlib import pyplot as plt
from matplotlib import patches
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

    # Wait for first messages
    while localPath is None or nextLocalWaypoint is None or nextGlobalWaypoint is None:
        rospy.loginfo("Waiting to receive first ROS messages")
        time.sleep(1)
    rospy.loginfo("ROS message received. Starting visualization")

    # Convert values from latlon to XY, relative to the nextGlobalWaypoint
    state = sailbot.getCurrentState()
    positionXY = latlonToXY(state.position, nextGlobalWaypoint)
    nextGlobalWaypointXY = latlonToXY(nextGlobalWaypoint, nextGlobalWaypoint)
    nextLocalWaypointXY = latlonToXY(nextLocalWaypoint, nextGlobalWaypoint)
    localPathXY = [latlonToXY(localWaypoint, nextGlobalWaypoint) for localWaypoint in localPath]
    localPathX = [xy[0] for xy in localPathXY]
    localPathY = [xy[1] for xy in localPathXY]
    shipsXY = [latlonToXY(latlon(ship.lat, ship.lon), nextGlobalWaypoint) for ship in state.AISData.ships]

    # Create plot with waypoints and boat
    axes = plt.gca()
    localPathPlot, = axes.plot(localPathX, localPathY, marker='.', color='g', markersize=10, linewidth=2)                        # Small green dots
    nextGlobalWaypointPlot, = axes.plot(nextGlobalWaypointXY[0], nextGlobalWaypointXY[1], marker='*', color='y', markersize=20)  # Yellow start
    nextLocalWaypointPlot, = axes.plot(nextLocalWaypointXY[0], nextLocalWaypointXY[1], marker='X', color='g', markersize=20)     # Green X
    positionPlot, = axes.plot(positionXY[0], positionXY[1], marker=(3,0,state.heading - 90), color='r', markersize=20)           # Blue arrow with correct heading

    # Setup plot xy limits and labels
    xPLim, xNLim, yPLim, yNLim = getXYLimits(localPathXY[0], nextGlobalWaypointXY)
    axes.set_xlim(xNLim, xPLim)
    axes.set_ylim(yNLim, yPLim)
    plt.grid(True)
    axes.set_xlabel('X distance to next global waypoint (km)')
    axes.set_ylabel('Y distance to next global waypoint (km)')

    # Show boats
    for ship in shipsXY:
        axes.add_patch(plt.Circle((ship[0], ship[1]), radius=100))                                                               # Multiple circles

    # Show wind direction arrow and wind speed text
    arrowLength = min(xPLim - xNLim, yPLim - yNLim) / 8
    arrowCenter = (xNLim + 1.5*arrowLength, yPLim - 1.5*arrowLength)
    windSpeedText = axes.text(arrowCenter[0], arrowCenter[1] + 1.1*arrowLength, "Wind Speed: {}".format(state.windSpeed), ha='center')
    arrowStart = (arrowCenter[0] - 0.5*arrowLength*math.cos(state.windDirection), arrowCenter[1] - 0.5*arrowLength*math.sin(state.windDirection))
    windDirection = patches.FancyArrow(arrowStart[0], arrowStart[1], arrowLength*math.cos(state.windDirection), arrowLength*math.sin(state.windDirection), width=arrowLength / 4)
    axes.add_patch(windDirection)

    while not rospy.is_shutdown():
        state = sailbot.getCurrentState()

        # Convert values from latlon to XY, relative to the nextGlobalWaypoint
        positionXY = latlonToXY(state.position, nextGlobalWaypoint)
        nextGlobalWaypointXY = latlonToXY(nextGlobalWaypoint, nextGlobalWaypoint)
        nextLocalWaypointXY = latlonToXY(nextLocalWaypoint, nextGlobalWaypoint)
        localPathXY = [latlonToXY(localWaypoint, nextGlobalWaypoint) for localWaypoint in localPath]
        localPathX = [xy[0] for xy in localPathXY]
        localPathY = [xy[1] for xy in localPathXY]
        shipsXY = [latlonToXY(latlon(ship.lat, ship.lon), nextGlobalWaypoint) for ship in state.AISData.ships]

        # Update plots
        localPathPlot.set_xdata(localPathX)
        localPathPlot.set_ydata(localPathY)
        nextGlobalWaypointPlot.set_xdata(nextGlobalWaypointXY[0])
        nextGlobalWaypointPlot.set_ydata(nextGlobalWaypointXY[1])
        nextLocalWaypointPlot.set_xdata(nextLocalWaypointXY[0])
        nextLocalWaypointPlot.set_ydata(nextLocalWaypointXY[1])
        positionPlot.set_xdata(positionXY[0])
        positionPlot.set_ydata(positionXY[1])
        positionPlot.set_marker((3, 0, state.heading-90))  # Creates a triangle with correct 'heading'

        # Removes all ships and wind arrow
        for p in axes.patches:
            p.remove()

        # Add boats
        for ship in shipsXY:
            axes.add_patch(plt.Circle((ship[0], ship[1]), radius=100))

        # Show wind direction arrow and wind speed text
        xPLim, xNLim, yPLim, yNLim = getXYLimits(localPathXY[0], nextGlobalWaypointXY)
        arrowLength = min(xPLim - xNLim, yPLim - yNLim) / 8
        arrowCenter = (xNLim + 1.5*arrowLength, yPLim - 1.5*arrowLength)
        windSpeedText.set_position((arrowCenter[0], arrowCenter[1] + 1.1*arrowLength))
        windSpeedText.set_text("Wind Speed: {}".format(state.windSpeed))
        arrowStart = (arrowCenter[0] - 0.5*arrowLength*math.cos(state.windDirection), arrowCenter[1] - 0.5*arrowLength*math.sin(state.windDirection))
        windDirection = patches.FancyArrow(arrowStart[0], arrowStart[1], arrowLength*math.cos(state.windDirection), arrowLength*math.sin(state.windDirection), width=arrowLength/4)
        axes.add_patch(windDirection)

        # Draw then sleep
        plt.draw()
        plt.pause(0.001)
        r.sleep()
