#!/usr/bin/env python
import sys
import rospy
import math
from Path import getPerpLine, WAYPOINT_REACHED_DISTANCE
# from main_loop import previousGlobalWaypoint
from std_msgs.msg import Float64
from sailbot_msg.msg import path, latlon
import utilities as utils
import obstacles as obs
import Sailbot as sbot
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
        yPLim = xWidth / yWidth * yPLim
        yNLim = xWidth / yWidth * yNLim
    else:
        xPLim = yWidth / xWidth * xPLim
        xNLim = yWidth / xWidth * xNLim

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
    positionOutOfBounds = outOfBounds(positionXY, xPLim, xNLim, yPLim, yNLim)
    nextGlobalWaypointOutOfBounds = outOfBounds(nextGlobalWaypointXY, xPLim, xNLim, yPLim, yNLim)
    if positionOutOfBounds or nextGlobalWaypointOutOfBounds:
        return True

    # Check if figure is too far zoomed out
    currentWidth = xPLim - xNLim
    currentHeight = yPLim - yNLim
    properWidth = math.fabs(nextGlobalWaypointXY[0] - positionXY[0])
    properHeight = math.fabs(nextGlobalWaypointXY[1] - positionXY[1])
    if max(properWidth / currentWidth, properHeight / currentHeight) < 0.3:
        return True

    return False

def getPerpPlot(isStartEast, slope, y_intercept, destinationXY):
    if slope == 0:
        return [destinationXY[0] - 5, destinationXY[0] + 5], [y_intercept, y_intercept]
    elif not slope:
        offset = WAYPOINT_REACHED_DISTANCE if isStartEast else -1 * WAYPOINT_REACHED_DISTANCE
        return [destinationXY[0] + offset, destinationXY[0] + offset],
        [destinationXY[1] - 5, destinationXY[1] + 5]
    else:
        # methods for boundary line
        def y(x):
            return slope * x + y_intercept

        # find the interception between the boundary line and path to the next waypoint
        # solve for x in y = slope*x + y_intercept = -1/m(x-nextX) + nextY
        centerX = (destinationXY[0] / slope + destinationXY[1] - y_intercept) / (slope + 1 / slope)

        deltaX = 5 * math.cos(math.atan(math.fabs(slope)))
        lineX = [centerX - deltaX, centerX + deltaX]
        
        # #Print statements for debugging:
        # rospy.logwarn("Method lines: ({}, {}); {}; {}".format(centerX, centerX,
        #                [centerX - deltaX, centerX + deltaX], [y(centerX - deltaX), y(centerX + deltaX)]))
        # rospy.logwarn('slope, y-intercept: ({}, {})'.format(slope, y_intercept))
        # rospy.logwarn('previous waypoint: ({}, {})'.format(localPathXY[prevInd][0], localPathXY[prevInd][1]))
        # rospy.logwarn('next waypoint: ({}, {})'.format(nextLocalWaypointXY[0], nextLocalWaypointXY[1]))
        # rospy.logwarn('boundary line middle: ({}, {})'.format(centerX, y(centerX)))
        # rospy.logwarn('current position: ({}, {})'.format(positionXY[0], positionXY[1]))
        # rospy.logwarn('x bounds 1: ({}, {})'.format(centerX - deltaX, y(centerX - deltaX)))
        # rospy.logwarn('x bounds 2: ({}, {})'.format(centerX + deltaX, y(centerX + deltaX)))
        
        return lineX, [y(lineX[0]), y(lineX[1])]            


if __name__ == '__main__':
    # Setup ros subscribers
    sailbot = sbot.Sailbot(nodeName='localPathVisualizer')
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
    positionXY = utils.latlonToXY(state.position, referenceLatlon)
    nextGlobalWaypointXY = utils.latlonToXY(nextGlobalWaypoint, referenceLatlon)
    # previousGlobalWaypointXY = utils.latlonToXY(previousGlobalWaypoint, referenceLatlon)
    nextLocalWaypointXY = utils.latlonToXY(nextLocalWaypoint, referenceLatlon)
    localPathXY = [utils.latlonToXY(localWaypoint, referenceLatlon) for localWaypoint in localPath]
    localPathX = [xy[0] for xy in localPathXY]
    localPathY = [xy[1] for xy in localPathXY]

    distances = [math.sqrt(((xy[0] - nextLocalWaypointXY[0])**2) + ((xy[1] - nextLocalWaypointXY[1])**2))
                 for xy in localPathXY]
    nextInd = distances.index(min(distances))
    prevInd = 0 if nextInd == 0 else nextInd - 1

    _, isStartEast, slope, y_intercept = getPerpLine(localPathXY[prevInd][0], localPathXY[prevInd][1],
                                                     nextLocalWaypointXY[0], nextLocalWaypointXY[1])

    # Create plot with waypoints and boat
    xPLim, xNLim, yPLim, yNLim = getXYLimits(positionXY, nextGlobalWaypointXY)
    markersize = min(xPLim - xNLim, yPLim - yNLim) / 2
    axes = plt.gca()
    localPathPlot, = axes.plot(localPathX, localPathY,
                               marker='.', color='g', markersize=markersize / 2, linewidth=2)  # Small green dots
    nextGlobalWaypointPlot, = axes.plot(nextGlobalWaypointXY[0], nextGlobalWaypointXY[1],
                                        marker='*', color='y', markersize=markersize)          # Yellow star
    nextLocalWaypointPlot, = axes.plot(nextLocalWaypointXY[0], nextLocalWaypointXY[1],
                                       marker='X', color='g', markersize=markersize)           # Green X

    lineX, lineY = getPerpPlot(isStartEast, slope, y_intercept, nextLocalWaypointXY)
    waypointReachedPlot, = axes.plot(lineX, lineY)

    # Red triangle with correct heading. The (-90) is because the triangle
    # default heading 0 points North, but this heading has 0 be East.
    positionPlot, = axes.plot(positionXY[0], positionXY[1],
                              marker=(3, 0, state.headingDegrees - 90), color='r', markersize=markersize)

    # Making the tail half of the visualizer
    positionPlotTail, = axes.plot(positionXY[0] - 1 * math.cos(math.radians(state.headingDegrees)),
                                  positionXY[1] - 1 * math.sin(math.radians(state.headingDegrees)),
                                  marker=(5, 0, state.headingDegrees - 90), color='r', markersize=.7 * markersize)

    # Setup plot xy limits and labels
    axes.set_xlim(xNLim, xPLim)
    axes.set_ylim(yNLim, yPLim)
    plt.grid(True)
    axes.set_xlabel('X distance to next global waypoint (km)')
    axes.set_ylabel('Y distance to next global waypoint (km)')
    axes.set_title('Local Path Visualizer (speedup = {})'.format(speedup))
    axes.set_aspect(aspect=1)

    fractionOfPlotLength = min(xPLim - xNLim, yPLim - yNLim) / 15
    arrowLength = fractionOfPlotLength

    # Show wind speed text and ocean current speed text
    windArrowCenter = (xNLim + 1.5 * arrowLength, yPLim - 1.5 * arrowLength)  # Upper left
    globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
        state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
    windSpeedText = axes.text(windArrowCenter[0], windArrowCenter[1] + 1.5 * fractionOfPlotLength,
                              "Global Wind Speed Kmph: {}".format(globalWindSpeedKmph), ha='center')

    oceanCurrentArrowCenter = (xPLim - 1.5 * arrowLength, yPLim - 1.5 * arrowLength)  # Upper right
    oceanCurrentSpeedText = axes.text(oceanCurrentArrowCenter[0],
                                      oceanCurrentArrowCenter[1] + 1.5 * fractionOfPlotLength,
                                      "Ocean Current Speed Kmph: {}"
                                      .format(rospy.get_param("ocean_current_speed", default=0)), ha='center')

    # Show position and global waypoint text
    positionLatlonText = axes.text(positionXY[0], positionXY[1] + 0.5 * fractionOfPlotLength,
                                   "(Lat: {}, Lon: {})".format(round(state.position.lat, LATLON_TEXT_DECIMAL_PLACES),
                                                               round(state.position.lon, LATLON_TEXT_DECIMAL_PLACES)),
                                   ha='center')
    nextGlobalWaypointLatlonText = axes.text(nextGlobalWaypointXY[0],
                                             nextGlobalWaypointXY[1] + 0.5 * fractionOfPlotLength,
                                             "(Lat: {}, Lon: {})"
                                             .format(round(nextGlobalWaypoint.lat, LATLON_TEXT_DECIMAL_PLACES),
                                                     round(nextGlobalWaypoint.lon, LATLON_TEXT_DECIMAL_PLACES)),
                                             ha='center')

    while not rospy.is_shutdown():
        state = sailbot.getCurrentState()
        referenceLatlon = nextGlobalWaypoint  # Ensure this matches createLocalPathSS referenceLatlon for best results

        # Convert values from latlon to XY, relative to the referenceLatlon
        positionXY = utils.latlonToXY(state.position, referenceLatlon)
        nextGlobalWaypointXY = utils.latlonToXY(nextGlobalWaypoint, referenceLatlon)
        nextLocalWaypointXY = utils.latlonToXY(nextLocalWaypoint, referenceLatlon)
        localPathXY = [utils.latlonToXY(localWaypoint, referenceLatlon) for localWaypoint in localPath]
        localPathX = [xy[0] for xy in localPathXY]
        localPathY = [xy[1] for xy in localPathXY]
        shipsXY = obs.getObstacles(state, referenceLatlon)

        distances = [math.sqrt(((xy[0] - nextLocalWaypointXY[0])**2) + ((xy[1] - nextLocalWaypointXY[1])**2))
                     for xy in localPathXY]
        nextInd = distances.index(min(distances))
        prevInd = 0 if nextInd == 0 else nextInd - 1

        _, isStartEast, slope, y_intercept = getPerpLine(localPathXY[prevInd][0], localPathXY[prevInd][1],
                                                         nextLocalWaypointXY[0], nextLocalWaypointXY[1])

        # Update plots
        localPathPlot.set_xdata(localPathX)
        localPathPlot.set_ydata(localPathY)
        nextGlobalWaypointPlot.set_xdata(nextGlobalWaypointXY[0])
        nextGlobalWaypointPlot.set_ydata(nextGlobalWaypointXY[1])
        nextLocalWaypointPlot.set_xdata(nextLocalWaypointXY[0])
        nextLocalWaypointPlot.set_ydata(nextLocalWaypointXY[1])
        positionPlot.set_xdata(positionXY[0])
        positionPlot.set_ydata(positionXY[1])
        positionPlot.set_marker((3, 0, state.headingDegrees - 90))  # Creates a triangle with correct 'heading'
        positionPlotTail.set_xdata(positionXY[0] - 1 * math.cos(math.radians(state.headingDegrees)))
        positionPlotTail.set_ydata(positionXY[1] - 1 * math.sin(math.radians(state.headingDegrees)))
        positionPlotTail.set_marker((5, 0, state.headingDegrees - 90))  # Creates a tail for the visualizer

        lineX, lineY = getPerpPlot(isStartEast, slope, y_intercept, nextLocalWaypointXY)
        waypointReachedPlot.set_xdata(lineX)
        waypointReachedPlot.set_ydata(lineY)

        # Resize axes if needed
        if needAxesResized(positionXY, nextGlobalWaypointXY, xPLim, xNLim, yPLim, yNLim):
            xPLim, xNLim, yPLim, yNLim = getXYLimits(positionXY, nextGlobalWaypointXY)
            axes.set_xlim(xNLim, xPLim)
            axes.set_ylim(yNLim, yPLim)

        fractionOfPlotLength = min(xPLim - xNLim, yPLim - yNLim) / 15
        arrowLength = fractionOfPlotLength

        # Update wind speed text
        windArrowCenter = (xNLim + 1.5 * arrowLength, yPLim - 1.5 * arrowLength)
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            state.measuredWindSpeedKmph, state.measuredWindDirectionDegrees, state.speedKmph, state.headingDegrees)
        windSpeedText.set_position((windArrowCenter[0], windArrowCenter[1] + 1.5 * fractionOfPlotLength))
        windSpeedText.set_text("Wind Speed Kmph: {}".format(globalWindSpeedKmph))

        # Update ocean current speed text
        oceanCurrentArrowCenter = (xPLim - 1.5 * arrowLength, yPLim - 1.5 * arrowLength)
        oceanCurrentSpeedText.set_position((oceanCurrentArrowCenter[0],
                                            oceanCurrentArrowCenter[1] + 1.5 * fractionOfPlotLength))
        oceanCurrentSpeedText.set_text("Ocean Current Speed Kmph: {}"
                                       .format(rospy.get_param("ocean_current_speed", default=0)))

        # Update position and global waypoint text
        positionLatlonText.set_position((positionXY[0], positionXY[1] + 0.5 * fractionOfPlotLength))
        positionLatlonText.set_text("(Lat: {}, Lon: {})".format(round(state.position.lat, LATLON_TEXT_DECIMAL_PLACES),
                                                                round(state.position.lon, LATLON_TEXT_DECIMAL_PLACES)))
        nextGlobalWaypointLatlonText.set_position(
            (nextGlobalWaypointXY[0], nextGlobalWaypointXY[1] + 0.5 * fractionOfPlotLength))
        nextGlobalWaypointLatlonText.set_text("(Lat: {}, Lon: {})"
                                              .format(round(nextGlobalWaypoint.lat, LATLON_TEXT_DECIMAL_PLACES),
                                                      round(nextGlobalWaypoint.lon, LATLON_TEXT_DECIMAL_PLACES)))

        # Update speedup text
        axes.set_title('Local Path Visualizer (speedup = {})'.format(speedup))

        # Add boats and wind speed arrow and ocean current arrow
        for ship in shipsXY:
            ship.addPatch(axes)
        globalWindDirectionRadians = math.radians(globalWindDirectionDegrees)
        windArrowStart = (windArrowCenter[0] - 0.5 * fractionOfPlotLength * math.cos(globalWindDirectionRadians),
                          windArrowCenter[1] - 0.5 * fractionOfPlotLength * math.sin(globalWindDirectionRadians))
        windDirection = patches.FancyArrow(windArrowStart[0], windArrowStart[1],
                                           fractionOfPlotLength * math.cos(math.radians(globalWindDirectionDegrees)),
                                           fractionOfPlotLength * math.sin(math.radians(globalWindDirectionDegrees)),
                                           width=fractionOfPlotLength / 4)
        axes.add_patch(windDirection)

        oceanCurrentDirectionRadians = math.radians(rospy.get_param('ocean_current_direction', default=0))
        oceanCurrentArrowStart = ((oceanCurrentArrowCenter[0] -
                                   0.5 * fractionOfPlotLength * math.cos(oceanCurrentDirectionRadians)),
                                  (oceanCurrentArrowCenter[1] -
                                   0.5 * fractionOfPlotLength * math.sin(oceanCurrentDirectionRadians)))
        oceanCurrentDirection = patches.FancyArrow(oceanCurrentArrowStart[0], oceanCurrentArrowStart[1],
                                                   fractionOfPlotLength * math.cos(oceanCurrentDirectionRadians),
                                                   fractionOfPlotLength * math.sin(oceanCurrentDirectionRadians),
                                                   width=fractionOfPlotLength / 4)
        axes.add_patch(oceanCurrentDirection)

        # Draw then sleep
        plt.draw()
        plt.pause(0.001)
        r.sleep()
        # Removes all ships and wind arrow and ocean current arrow
        for p in axes.patches:
            p.remove()
