#!/usr/bin/env python
import rospy
import math
import local_pathfinding.msg as msg
from local_pathfinding.msg import AIS_msg, GPS, path, latlon, wind
from utilities import *
from Sailbot import *
from matplotlib import pyplot as plt

localPath = []
nextLocalWaypoint = latlon(0, 0)
nextGlobalWaypoint = latlon(0, 0)

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
    xPLim = 10
    xNLim = -10
    yPLim = 10
    yNLim = -10
    plt.axis([xNLim, xPLim, yNLim, xPLim])
    plt.ion()
    plt.show()

    while not rospy.is_shutdown():
        rospy.loginfo("nextLocalWaypoint at beginning: {}".format(nextLocalWaypoint))
        rospy.loginfo("nextGlobalWaypoint at beginning: {}".format(nextGlobalWaypoint))
        state = sailbot.getCurrentState()
        positionXY = [0, 0]
        nextGlobalWaypointXY = latlonToXY(nextGlobalWaypoint, state.position)
        nextLocalWaypointXY = latlonToXY(nextLocalWaypoint, state.position)
        rospy.loginfo("nextLocalWaypointXY at beginning: {}".format(nextLocalWaypointXY))
        rospy.loginfo("nextGlobalWaypointXY at beginning: {}".format(nextGlobalWaypointXY))
        localPathXY = [latlonToXY(localWaypoint, state.position) for localWaypoint in localPath]
        shipsXY = [latlonToXY(latlon(ship.lat, ship.lon), state.position) for ship in state.AISData.ships]

        # Keep same axis xy limits unless change is needed
        newXPLim = max([positionXY[0], nextGlobalWaypointXY[0]])
        newXNLim = min([positionXY[0], nextGlobalWaypointXY[0]])
        newYPLim = max([positionXY[1], nextGlobalWaypointXY[1]])
        newYNLim = min([positionXY[1], nextGlobalWaypointXY[1]])
        extra = 1000
        if newXPLim > xPLim or newXNLim < xNLim or newYPLim > yPLim or newYNLim < yNLim:
            xPLim = newXPLim
            xNLim = newXNLim
            yPLim = newYPLim
            yNLim = newYNLim

        plt.xlim([xNLim-extra, xPLim+extra])
        plt.ylim([yNLim-extra, yPLim+extra])

        # Plot sailbot, next local waypoint, next global waypoint
        rospy.loginfo("PostionXY = {}. nextGlobalWaypointXY = {}. nextLocalWaypointXY = {}".format(positionXY, nextGlobalWaypointXY, nextLocalWaypointXY))
        plt.plot(positionXY[0], positionXY[1], marker='o', color='b')
        plt.plot(nextGlobalWaypointXY[0], nextGlobalWaypointXY[1], marker='*', color='y')
        plt.plot(nextLocalWaypointXY[0], nextLocalWaypointXY[1], marker='*', color='g')

        # Plot local path
        localPathX = [xy[0] for xy in localPathXY]
        localPathY = [xy[1] for xy in localPathXY]
        plt.plot(localPathX, localPathY, marker='*', color='g')

        # Plot AIS
        # for ship in shipsXY:
            # plt.add_patch(plt.Circle((ship[0], ship[1]), radius=1))

        plt.draw()
        plt.pause(0.001)

        # Sleep then clear plot
        r.sleep()
        plt.cla()
