#!/usr/bin/env python
import rospy
import math
from Path import getPerpLine, WAYPOINT_REACHED_DISTANCE
from sailbot_msg.msg import path, latlon
import utilities as utils
import obstacles as obs
import Sailbot as sbot
from matplotlib import pyplot as plt
from matplotlib import patches

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


def previousGlobalWaypointCallback(data):
    global previousGlobalWaypoint
    previousGlobalWaypoint = data


def previousLocalWaypointCallback(data):
    global previousLocalWaypoint
    previousLocalWaypoint = data


def getXYLimits(xy0, xy1):
    # Set xy for figure
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
    destinationX, destinationY = destinationXY

    if slope == 0:
        return [destinationX - 5, destinationX + 5], [y_intercept, y_intercept]
    elif not slope:
        offset = WAYPOINT_REACHED_DISTANCE if isStartEast else -1 * WAYPOINT_REACHED_DISTANCE
        return [destinationX + offset, destinationX + offset], [destinationY - 5, destinationY + 5]
    else:
        # methods for boundary line
        def y(x):
            return slope * x + y_intercept

        # find the interception between the boundary line and path to the next waypoint
        # solve for x in y = slope*x + y_intercept = -1/m(x-nextX) + nextY
        centerX = (destinationX / slope + destinationY - y_intercept) / (slope + 1 / slope)

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
    rospy.Subscriber("previousLocalWaypoint", latlon, previousLocalWaypointCallback)
    rospy.Subscriber("nextGlobalWaypoint", latlon, nextGlobalWaypointCallback)
    rospy.Subscriber("previousGlobalWaypoint", latlon, previousGlobalWaypointCallback)
    r = rospy.Rate(1.0 / VISUALIZER_UPDATE_PERIOD_SECONDS)

    sailbot.waitForFirstSensorDataAndGlobalPath()
    rospy.loginfo("ROS message received. Starting visualization")

    state = sailbot.getCurrentState()
    referenceLatlon = state.position

    positionXY = utils.latlonToXY(state.position, referenceLatlon)

    # Convert values from latlon to XY, relative to the referenceLatlon
    if nextGlobalWaypoint:  # check that nextGlobalWaypoint is available
        nextGlobalWaypointXY = utils.latlonToXY(nextGlobalWaypoint, referenceLatlon)
        previousGlobalWaypointXY = utils.latlonToXY(previousGlobalWaypoint, referenceLatlon)
        _, glob_isStartEast, glob_slope, glob_y = getPerpLine(previousGlobalWaypointXY, nextGlobalWaypointXY, True)

    if nextLocalWaypoint:
        nextLocalWaypointXY = utils.latlonToXY(nextLocalWaypoint, referenceLatlon)
        previousLocalWaypointXY = utils.latlonToXY(previousLocalWaypoint, referenceLatlon)
        _, isStartEast, slope, y_intercept = getPerpLine(previousLocalWaypointXY, nextLocalWaypointXY)

    if localPath:
        localPathXY = [utils.latlonToXY(localWaypoint, referenceLatlon) for localWaypoint in localPath]
        localPathX = [xy[0] for xy in localPathXY]
        localPathY = [xy[1] for xy in localPathXY]

    # Create plot with waypoints and boat
    # Initialize plot limits to hardcoded values if nextGlobalWaypointXY has not yet been initialized
    xPLim, xNLim, yPLim, yNLim = getXYLimits(positionXY, nextGlobalWaypointXY) \
        if 'nextGlobalWaypointXY' in globals() else (30.0, 10.0, 30.0, 10.0)
    markersize = min(xPLim - xNLim, yPLim - yNLim) / 2
    axes = plt.gca()

    # Plot if the required variables are defined
    if 'nextGlobalWaypointXY' in globals():
        nextGlobalWaypointPlot, = axes.plot(nextGlobalWaypointXY[0], nextGlobalWaypointXY[1],
                                            marker='*', color='y', markersize=markersize)  # Yellow star
        glob_X, glob_Y = getPerpPlot(glob_isStartEast, glob_slope, glob_y, nextGlobalWaypointXY)
        globalWaypointReachedPlot, = axes.plot(glob_X, glob_Y, color='r')

    if 'nextLocalWaypointXY' in globals():
        nextLocalWaypointPlot, = axes.plot(nextLocalWaypointXY[0], nextLocalWaypointXY[1],
                                           marker='X', color='g', markersize=markersize)  # Green X
        lineX, lineY = getPerpPlot(isStartEast, slope, y_intercept, nextLocalWaypointXY)
        waypointReachedPlot, = axes.plot(lineX, lineY, color='b')

    if 'localPathXY' in globals():
        localPathPlot, = axes.plot(localPathX, localPathY, marker='.', color='g', markersize=markersize / 2,
                                   linewidth=2)  # Green dots

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
    axes.set_title('Local Path Visualizer (speedup = {})'.format(rospy.get_param('speedup', default=1.0)))
    axes.set_aspect(aspect=1)

    fractionOfPlotLen = min(xPLim - xNLim, yPLim - yNLim) / 15
    arrowLength = fractionOfPlotLen

    # Show wind speed text and ocean current speed text
    windArrowCenter = (xNLim + 1.5 * arrowLength, yPLim - 1.5 * arrowLength)  # Upper left
    windSpeedText = axes.text(windArrowCenter[0], windArrowCenter[1] + 1.5 * fractionOfPlotLen,
                              "Global Wind Speed Kmph: {}".format(state.globalWindSpeedKmph), ha='center')

    oceanCurrentArrowCenter = (xPLim - 1.5 * arrowLength, yPLim - 1.5 * arrowLength)  # Upper right
    oceanCurrentSpeedText = axes.text(oceanCurrentArrowCenter[0],
                                      oceanCurrentArrowCenter[1] + 1.5 * fractionOfPlotLen,
                                      "Ocean Current Speed Kmph: {}"
                                      .format(rospy.get_param("ocean_current_speed", default=0)), ha='center')

    # Show position and global waypoint text if they required variables are defined
    if 'nextGlobalWaypointXY' in globals():
        nextGlobalWaypointLatlonText = axes.text(nextGlobalWaypointXY[0],
                                                 nextGlobalWaypointXY[1] + 0.5 * fractionOfPlotLen,
                                                 "(Lat: {}, Lon: {})"
                                                 .format(round(nextGlobalWaypoint.lat, LATLON_TEXT_DECIMAL_PLACES),
                                                         round(nextGlobalWaypoint.lon, LATLON_TEXT_DECIMAL_PLACES)),
                                                 ha='center')

    positionLatlonText = axes.text(positionXY[0], positionXY[1] + 0.5 * fractionOfPlotLen,
                                   "(Lat: {}, Lon: {}) {} kmph"
                                   .format(round(state.position.lat, LATLON_TEXT_DECIMAL_PLACES),
                                           round(state.position.lon, LATLON_TEXT_DECIMAL_PLACES),
                                           round(state.speedKmph, LATLON_TEXT_DECIMAL_PLACES)), ha='center')

    while not rospy.is_shutdown():
        state = sailbot.getCurrentState()
        referenceLatlon = state.position

        positionXY = utils.latlonToXY(state.position, referenceLatlon)

        # Convert values from latlon to XY, relative to the referenceLatlon
        if nextGlobalWaypoint:
            nextGlobalWaypointXY = utils.latlonToXY(nextGlobalWaypoint, referenceLatlon)
            previousGlobalWaypointXY = utils.latlonToXY(previousGlobalWaypoint, referenceLatlon)
            _, glob_isStartEast, glob_slope, glob_y = getPerpLine(previousGlobalWaypointXY, nextGlobalWaypointXY, True)

        if nextLocalWaypoint:
            nextLocalWaypointXY = utils.latlonToXY(nextLocalWaypoint, referenceLatlon)
            previousLocalWaypointXY = utils.latlonToXY(previousLocalWaypoint, referenceLatlon)
            _, isStartEast, slope, y_intercept = getPerpLine(previousLocalWaypointXY, nextLocalWaypointXY)

        if localPath:
            localPathXY = [utils.latlonToXY(localWaypoint, referenceLatlon) for localWaypoint in localPath]
            localPathX = [xy[0] for xy in localPathXY]
            localPathY = [xy[1] for xy in localPathXY]

        shipsXY = obs.getObstacles(state, referenceLatlon)

        # Update plots, or plot them if they don't yet exist and the required variables are defined
        if 'nextGlobalWaypointXY' in globals():
            # Resize axes and markers if needed
            if needAxesResized(positionXY, nextGlobalWaypointXY, xPLim, xNLim, yPLim, yNLim):
                xPLim, xNLim, yPLim, yNLim = getXYLimits(positionXY, nextGlobalWaypointXY)
                axes.set_xlim(xNLim, xPLim)
                axes.set_ylim(yNLim, yPLim)
                markersize = min(xPLim - xNLim, yPLim - yNLim) / 2

            glob_X, glob_Y = getPerpPlot(glob_isStartEast, glob_slope, glob_y, nextGlobalWaypointXY)
            if 'nextGlobalWaypointPlot' in globals():
                nextGlobalWaypointPlot.set_xdata(nextGlobalWaypointXY[0])
                nextGlobalWaypointPlot.set_ydata(nextGlobalWaypointXY[1])
                globalWaypointReachedPlot.set_xdata(glob_X)
                globalWaypointReachedPlot.set_ydata(glob_Y)
            else:
                nextGlobalWaypointPlot, = axes.plot(nextGlobalWaypointXY[0], nextGlobalWaypointXY[1],
                                                    marker='*', color='y', markersize=markersize)  # Yellow star
                globalWaypointReachedPlot, = axes.plot(glob_X, glob_Y, color='r')

        if 'nextLocalWaypointXY' in globals():
            lineX, lineY = getPerpPlot(isStartEast, slope, y_intercept, nextLocalWaypointXY)
            if 'nextLocalWaypointPlot' in globals():
                nextLocalWaypointPlot.set_xdata(nextLocalWaypointXY[0])
                nextLocalWaypointPlot.set_ydata(nextLocalWaypointXY[1])
                waypointReachedPlot.set_xdata(lineX)
                waypointReachedPlot.set_ydata(lineY)
            else:
                nextLocalWaypointPlot, = axes.plot(nextLocalWaypointXY[0], nextLocalWaypointXY[1],
                                                   marker='X', color='g', markersize=markersize)  # Green X
                waypointReachedPlot, = axes.plot(lineX, lineY, color='b')

        if 'localPathXY' in globals():
            if 'localPathPlot' in globals():
                localPathPlot.set_xdata(localPathX)
                localPathPlot.set_ydata(localPathY)
            else:
                localPathPlot, = axes.plot(localPathX, localPathY, marker='.', color='g', markersize=markersize / 2,
                                           linewidth=2)  # Green dots

        positionPlot.set_xdata(positionXY[0])
        positionPlot.set_ydata(positionXY[1])
        positionPlot.set_marker((3, 0, state.headingDegrees - 90))  # Creates a triangle with correct 'heading'
        positionPlotTail.set_xdata(positionXY[0] - 1 * math.cos(math.radians(state.headingDegrees)))
        positionPlotTail.set_ydata(positionXY[1] - 1 * math.sin(math.radians(state.headingDegrees)))
        positionPlotTail.set_marker((5, 0, state.headingDegrees - 90))  # Creates a tail for the visualizer

        fractionOfPlotLen = min(xPLim - xNLim, yPLim - yNLim) / 15
        arrowLength = fractionOfPlotLen

        # Update wind speed text
        windArrowCenter = (xNLim + 1.5 * arrowLength, yPLim - 1.5 * arrowLength)
        windSpeedText.set_position((windArrowCenter[0], windArrowCenter[1] + 1.5 * fractionOfPlotLen))
        windSpeedText.set_text("Wind Speed Kmph: {}".format(state.globalWindSpeedKmph))

        # Update ocean current speed text
        oceanCurrentArrowCenter = (xPLim - 1.5 * arrowLength, yPLim - 1.5 * arrowLength)
        oceanCurrentSpeedText.set_position((oceanCurrentArrowCenter[0],
                                            oceanCurrentArrowCenter[1] + 1.5 * fractionOfPlotLen))
        oceanCurrentSpeedText.set_text("Ocean Current Speed Kmph: {}"
                                       .format(rospy.get_param("ocean_current_speed", default=0)))

        # Update position and global waypoint text,
        # or show them if they don't yet exist and the required variables are defined
        if 'nextGlobalWaypointLatlonText' in globals():
            nextGlobalWaypointLatlonText.set_position(
                (nextGlobalWaypointXY[0], nextGlobalWaypointXY[1] + 0.5 * fractionOfPlotLen))
            nextGlobalWaypointLatlonText.set_text("(Lat: {}, Lon: {})"
                                                  .format(round(nextGlobalWaypoint.lat, LATLON_TEXT_DECIMAL_PLACES),
                                                          round(nextGlobalWaypoint.lon, LATLON_TEXT_DECIMAL_PLACES)))
        elif 'nextGlobalWaypointXY' in globals():
            nextGlobalWaypointLatlonText = axes.text(nextGlobalWaypointXY[0],
                                                     nextGlobalWaypointXY[1] + 0.5 * fractionOfPlotLen,
                                                     "(Lat: {}, Lon: {})"
                                                     .format(round(nextGlobalWaypoint.lat, LATLON_TEXT_DECIMAL_PLACES),
                                                             round(nextGlobalWaypoint.lon, LATLON_TEXT_DECIMAL_PLACES)),
                                                     ha='center')

        positionLatlonText.set_position((positionXY[0], positionXY[1] + 0.5 * fractionOfPlotLen))
        positionLatlonText.set_text("(Lat: {}, Lon: {}) {} kmph"
                                    .format(round(state.position.lat, LATLON_TEXT_DECIMAL_PLACES),
                                            round(state.position.lon, LATLON_TEXT_DECIMAL_PLACES),
                                            round(state.speedKmph, LATLON_TEXT_DECIMAL_PLACES)))

        # Update speedup text
        axes.set_title('Local Path Visualizer (speedup = {})'.format(rospy.get_param('speedup', default=1.0)))

        # Add boats and wind speed arrow and ocean current arrow
        for ship in shipsXY:
            ship.addPatch(axes)
        globalWindDirectionRadians = math.radians(state.globalWindDirectionDegrees)
        windArrowStart = (windArrowCenter[0] - 0.5 * fractionOfPlotLen * math.cos(globalWindDirectionRadians),
                          windArrowCenter[1] - 0.5 * fractionOfPlotLen * math.sin(globalWindDirectionRadians))
        windDirection = patches.FancyArrow(windArrowStart[0], windArrowStart[1],
                                           fractionOfPlotLen * math.cos(math.radians(state.globalWindDirectionDegrees)),
                                           fractionOfPlotLen * math.sin(math.radians(state.globalWindDirectionDegrees)),
                                           width=fractionOfPlotLen / 4)
        axes.add_patch(windDirection)

        oceanCurrentDirectionRadians = math.radians(rospy.get_param('ocean_current_direction', default=0))
        oceanCurrentArrowStart = ((oceanCurrentArrowCenter[0] -
                                   0.5 * fractionOfPlotLen * math.cos(oceanCurrentDirectionRadians)),
                                  (oceanCurrentArrowCenter[1] -
                                   0.5 * fractionOfPlotLen * math.sin(oceanCurrentDirectionRadians)))
        oceanCurrentDirection = patches.FancyArrow(oceanCurrentArrowStart[0], oceanCurrentArrowStart[1],
                                                   fractionOfPlotLen * math.cos(oceanCurrentDirectionRadians),
                                                   fractionOfPlotLen * math.sin(oceanCurrentDirectionRadians),
                                                   width=fractionOfPlotLen / 4)
        axes.add_patch(oceanCurrentDirection)

        # Draw then sleep
        plt.draw()
        plt.pause(0.001)
        r.sleep()
        # Removes all ships and wind arrow and ocean current arrow
        for p in axes.patches:
            p.remove()
