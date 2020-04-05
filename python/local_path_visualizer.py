#!/usr/bin/env python
import sys
import rospy
import math
from std_msgs.msg import Float64
from local_pathfinding.msg import AISMsg, GPS, path, latlon, windSensor
from utilities import *
from Sailbot import *
from matplotlib import pyplot as plt
from matplotlib import patches
import time

# Constants
VISUALIZER_UPDATE_PERIOD_SECONDS = 0.1
LATLON_TEXT_DECIMAL_PLACES = 3

# Globals for callbacks
localPath = None
nextLocalWaypoint = None
nextGlobalWaypoint = None

# ROS subscribe callbacks
def localPathCallback(data):
    global localPath
    localPath = data.waypoints

def nextLocalWaypointCallback(data):
    global nextLocalWaypoint
    nextLocalWaypoint = data

def nextGlobalWaypointCallback(data):
    global nextGlobalWaypoint
    nextGlobalWaypoint = data

# Global variable for speedup
speedup = 1.0
def speedupCallback(data):
    global speedup
    speedup = data.data

# Set xy for figure
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
    multiplier = 0.5
    extraWidth = multiplier * (xPLim - xNLim)
    xPLim += extraWidth
    xNLim -= extraWidth
    yPLim += extraWidth
    yNLim -= extraWidth
    return xPLim, xNLim, yPLim, yNLim

# Check when figure needs resizing
def needAxesResized(positionXY, nextGlobalWaypointXY, xPLim, xNLim, yPLim, yNLim):
    def outOfBounds(xy, xPLim, xNLim, yPLim, yNLim):
        return xy[0] < xNLim or xy[1] < yNLim or xy[0] > xPLim or xy[1] > yPLim

    # Check if boat or goal is out of bounds
    if outOfBounds(positionXY, xPLim, xNLim, yPLim, yNLim) or outOfBounds(nextGlobalWaypointXY, xPLim, xNLim, yPLim, yNLim):
        return True

    # Check if figure is too far zoomed out
    currentWidth = xPLim - xNLim
    currentHeight = yPLim - yNLim
    properWidth = math.fabs(nextGlobalWaypointXY[0] - positionXY[0])
    properHeight = math.fabs(nextGlobalWaypointXY[1] - positionXY[1])
    if max(properWidth / currentWidth, properHeight / currentHeight) < 0.3:
        return True

    return False

if __name__ == '__main__':
    # Setup ros subscribers
    sailbot = Sailbot(nodeName='localPathVisualizer')
    rospy.Subscriber("localPath", path, localPathCallback)
    rospy.Subscriber("nextLocalWaypoint", latlon, nextLocalWaypointCallback)
    rospy.Subscriber("nextGlobalWaypoint", latlon, nextGlobalWaypointCallback)
    rospy.Subscriber("speedup", Float64, speedupCallback)
    r = rospy.Rate(1.0 / VISUALIZER_UPDATE_PERIOD_SECONDS)

    # Wait for first messages
    while localPath is None or nextLocalWaypoint is None or nextGlobalWaypoint is None:
        # Exit if shutdown
        if rospy.is_shutdown():
            rospy.loginfo("rospy.is_shutdown() is True. Exiting")
            sys.exit()
        else:
            rospy.loginfo("Waiting to receive first ROS messages")
            time.sleep(1)
    rospy.loginfo("ROS message received. Starting visualization")

    # Convert values from latlon to XY, relative to the referenceLatlon
    state = sailbot.getCurrentState()
    referenceLatlon = nextGlobalWaypoint  # Ensure that this matches createLocalPathSS referenceLatlon for best results
    positionXY = latlonToXY(state.position, referenceLatlon)
    nextGlobalWaypointXY = latlonToXY(nextGlobalWaypoint, referenceLatlon)
    nextLocalWaypointXY = latlonToXY(nextLocalWaypoint, referenceLatlon)
    localPathXY = [latlonToXY(localWaypoint, referenceLatlon) for localWaypoint in localPath]
    localPathX = [xy[0] for xy in localPathXY]
    localPathY = [xy[1] for xy in localPathXY]

    # Create plot with waypoints and boat
    xPLim, xNLim, yPLim, yNLim = getXYLimits(positionXY, nextGlobalWaypointXY)
    markersize = min(xPLim - xNLim, yPLim - yNLim) / 2
    axes = plt.gca()
    localPathPlot, = axes.plot(localPathX, localPathY, marker='.', color='g', markersize=markersize / 2, linewidth=2)                    # Small green dots
    nextGlobalWaypointPlot, = axes.plot(nextGlobalWaypointXY[0], nextGlobalWaypointXY[1], marker='*', color='y', markersize=markersize)  # Yellow star
    nextLocalWaypointPlot, = axes.plot(nextLocalWaypointXY[0], nextLocalWaypointXY[1], marker='X', color='g', markersize=markersize)     # Green X
    positionPlot, = axes.plot(positionXY[0], positionXY[1], marker=(3,0,state.headingDegrees - 90), color='r', markersize=markersize)    # Red triangle with correct heading. The (-90) is because the triangle default heading 0 points North, but this heading has 0 be East.

    # Setup plot xy limits and labels
    axes.set_xlim(xNLim, xPLim)
    axes.set_ylim(yNLim, yPLim)
    plt.grid(True)
    axes.set_xlabel('X distance to next global waypoint (km)')
    axes.set_ylabel('Y distance to next global waypoint (km)')
    axes.set_title('Local Path Visualizer (speedup = {})'.format(speedup))
    axes.set_aspect(aspect=1)

    # Show wind speed text and position text
    arrowLength = min(xPLim - xNLim, yPLim - yNLim) / 15
    arrowCenter = (xNLim + 1.5*arrowLength, yPLim - 1.5*arrowLength)
    globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
    windSpeedText = axes.text(arrowCenter[0], arrowCenter[1] + 1.5*arrowLength, "Global Wind Speed Kmph: {}".format(globalWindSpeedKmph), ha='center')
    positionLatlonText = axes.text(positionXY[0], positionXY[1] + 0.5*arrowLength, "(Lat: {}, Lon: {})".format(round(state.position.lat, LATLON_TEXT_DECIMAL_PLACES), round(state.position.lon, LATLON_TEXT_DECIMAL_PLACES)), ha='center')
    nextGlobalWaypointLatlonText = axes.text(nextGlobalWaypointXY[0], nextGlobalWaypointXY[1] + 0.5*arrowLength, "(Lat: {}, Lon: {})".format(round(nextGlobalWaypoint.lat, LATLON_TEXT_DECIMAL_PLACES), round(nextGlobalWaypoint.lon, LATLON_TEXT_DECIMAL_PLACES)), ha='center')

    while not rospy.is_shutdown():
        state = sailbot.getCurrentState()
        referenceLatlon = nextGlobalWaypoint  # Ensure that this matches createLocalPathSS referenceLatlon for best results

        # Convert values from latlon to XY, relative to the referenceLatlon
        positionXY = latlonToXY(state.position, referenceLatlon)
        nextGlobalWaypointXY = latlonToXY(nextGlobalWaypoint, referenceLatlon)
        nextLocalWaypointXY = latlonToXY(nextLocalWaypoint, referenceLatlon)
        localPathXY = [latlonToXY(localWaypoint, referenceLatlon) for localWaypoint in localPath]
        localPathX = [xy[0] for xy in localPathXY]
        localPathY = [xy[1] for xy in localPathXY]
        shipsXY = getObstacles(state.AISData.ships, state.position, state.speedKmph, referenceLatlon)
            
        # Update plots
        localPathPlot.set_xdata(localPathX)
        localPathPlot.set_ydata(localPathY)
        nextGlobalWaypointPlot.set_xdata(nextGlobalWaypointXY[0])
        nextGlobalWaypointPlot.set_ydata(nextGlobalWaypointXY[1])
        nextLocalWaypointPlot.set_xdata(nextLocalWaypointXY[0])
        nextLocalWaypointPlot.set_ydata(nextLocalWaypointXY[1])
        positionPlot.set_xdata(positionXY[0])
        positionPlot.set_ydata(positionXY[1])
        positionPlot.set_marker((3, 0, state.headingDegrees-90))  # Creates a triangle with correct 'heading'

        # Resize axes if needed
        if needAxesResized(positionXY, nextGlobalWaypointXY, xPLim, xNLim, yPLim, yNLim):
            xPLim, xNLim, yPLim, yNLim = getXYLimits(positionXY, nextGlobalWaypointXY)
            axes.set_xlim(xNLim, xPLim)
            axes.set_ylim(yNLim, yPLim)

        # Update wind speed text
        arrowLength = min(xPLim - xNLim, yPLim - yNLim) / 15
        arrowCenter = (xNLim + 1.5*arrowLength, yPLim - 1.5*arrowLength)
        globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
        windSpeedText.set_position((arrowCenter[0], arrowCenter[1] + 1.5*arrowLength))
        windSpeedText.set_text("Wind Speed Kmph: {}".format(globalWindSpeedKmph))
        positionLatlonText.set_position((positionXY[0], positionXY[1] + 0.5*arrowLength))
        positionLatlonText.set_text("(Lat: {}, Lon: {})".format(round(state.position.lat, LATLON_TEXT_DECIMAL_PLACES), round(state.position.lon, LATLON_TEXT_DECIMAL_PLACES)))
        nextGlobalWaypointLatlonText.set_position((nextGlobalWaypointXY[0], nextGlobalWaypointXY[1] + 0.5*arrowLength))
        nextGlobalWaypointLatlonText.set_text("(Lat: {}, Lon: {})".format(round(nextGlobalWaypoint.lat, LATLON_TEXT_DECIMAL_PLACES), round(nextGlobalWaypoint.lon, LATLON_TEXT_DECIMAL_PLACES)))

        # Update speedup text
        axes.set_title('Local Path Visualizer (speedup = {})'.format(speedup))

        # Add boats and wind speed arrow
        for ship in shipsXY:
            ship.addPatch(axes)
        arrowStart = (arrowCenter[0] - 0.5*arrowLength*math.cos(math.radians(globalWindDirectionDegrees)), arrowCenter[1] - 0.5*arrowLength*math.sin(math.radians(globalWindDirectionDegrees)))
        windDirection = patches.FancyArrow(arrowStart[0], arrowStart[1], arrowLength*math.cos(math.radians(globalWindDirectionDegrees)), arrowLength*math.sin(math.radians(globalWindDirectionDegrees)), width=arrowLength/4)
        axes.add_patch(windDirection)

        # Draw then sleep
        plt.draw()
        plt.pause(0.001)
        r.sleep()
        # Removes all ships and wind arrow
        for p in axes.patches:
            p.remove()
