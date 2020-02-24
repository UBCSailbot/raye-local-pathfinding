#! /usr/bin/env python

# Add python directory to path
import sys, os
testdir = os.path.dirname(__file__)
srcdir = '../python'
sys.path.insert(0, os.path.abspath(os.path.join(testdir, srcdir)))

import unittest
import rostest
from local_pathfinding.msg import latlon, AISShip, AISMsg
from utilities import *
from Sailbot import BoatState
from geopy.distance import distance
from math import sqrt
import matplotlib.pyplot as plt

class TestUtilities(unittest.TestCase):
    def test_latlonToXY_and_XYToLatlon_advanced(self):
        # Setup latlons
        startLatlon = latlon(49.263022, -123.023447)
        endLatlon = latlon(47.7984, -125.3319)

        # Test latlonToXY
        xy = latlonToXY(latlon=endLatlon, referenceLatlon=startLatlon)
        x, y = xy
        self.assertAlmostEqual(x, -168.0158959716741, places=2)  # endLatlon is about 168.0158959716741km west of startLatlon
        self.assertAlmostEqual(y, -162.8668880228988, places=2)  # endLatLon is about 162.8668880228988km south of startLatlon

        # Test XYToLatlon
        calculatedEndLatlon = XYToLatlon(xy=xy, referenceLatlon=startLatlon)
        self.assertAlmostEqual(calculatedEndLatlon.lat, endLatlon.lat, places=1)  # calculatedEndLatlon should be same as endLatlon
        self.assertAlmostEqual(calculatedEndLatlon.lon, endLatlon.lon, places=1)

    def test_latlonToXY_and_XYToLatlon_basic(self):
        # Setup latlon
        testLatlon = latlon(50, -120)

        # Test latlonToXY
        xy = latlonToXY(latlon=testLatlon, referenceLatlon=testLatlon)
        x, y = xy
        self.assertAlmostEqual(x, 0, places=2)  # xy=(0,0) at reference point
        self.assertAlmostEqual(y, 0, places=2)

        # Test XYToLatlon
        calculatedLatlon = XYToLatlon(xy=xy, referenceLatlon=testLatlon)
        self.assertAlmostEqual(calculatedLatlon.lat, testLatlon.lat, places=1)  # calculatedLatlon should be same as testLatlon
        self.assertAlmostEqual(calculatedLatlon.lon, testLatlon.lon, places=1)

    def test_getDesiredHeading_north(self):
        # Heading is defined using cartesian coordinates. 0 degrees is East. 90 degrees in North. 270 degrees is South.
        # Bearing is defined differently. 0 degrees is North. 90 degrees is East. 180 degrees is South.
        # Heading = -Bearing + 90

        # Setup latlon
        position = latlon(50, -120)

        # Setup destination
        destination = distance(kilometers=1).destination(point=(position.lat, position.lon), bearing=BEARING_NORTH)
        localWaypoint = latlon(destination.latitude, destination.longitude)

        # Test desiredHeading
        desiredHeading = getDesiredHeading(position=position, localWaypoint=localWaypoint)
        self.assertAlmostEqual(desiredHeading, HEADING_NORTH, places=1)

    def test_getDesiredHeading_east(self):
        # Setup latlon
        position = latlon(50, -120)

        # Setup destination
        destination = distance(kilometers=1).destination(point=(position.lat, position.lon), bearing=BEARING_EAST)
        localWaypoint = latlon(destination.latitude, destination.longitude)

        # Test desiredHeading
        desiredHeading = getDesiredHeading(position=position, localWaypoint=localWaypoint)
        self.assertAlmostEqual(desiredHeading, HEADING_EAST, places=1)

    def test_globalWaypointReached(self):
        position = latlon(35, -150)

        # Far away globalWaypoint unreached
        unreachedPosition = distance(kilometers=2*GLOBAL_WAYPOINT_REACHED_RADIUS_KM).destination(point=(position.lat, position.lon), bearing=BEARING_EAST)
        unreachedGlobalWaypoint = latlon(unreachedPosition.latitude, unreachedPosition.longitude)
        self.assertFalse(globalWaypointReached(position, unreachedGlobalWaypoint))

        # Far away globalWaypoint reached
        reachedPosition = distance(kilometers=GLOBAL_WAYPOINT_REACHED_RADIUS_KM/2).destination(point=(position.lat, position.lon), bearing=BEARING_EAST)
        reachedGlobalWaypoint = latlon(reachedPosition.latitude, reachedPosition.longitude)
        self.assertTrue(globalWaypointReached(position, reachedGlobalWaypoint))

    def test_getLocalWaypointLatlon(self):
        # Empty localPath should give back default (0,0) latlon
        emptyLocalPathLatlon = getLocalWaypointLatLon(localPath=[], localPathIndex=1)
        self.assertAlmostEqual(emptyLocalPathLatlon.lat, 0, places=1)
        self.assertAlmostEqual(emptyLocalPathLatlon.lon, 0, places=1)

        localPath = [latlon(48, -124), latlon(38, -134), latlon(28, -144), latlon(22, -150)]

        # Test index out of bounds. Should give back last latlon in path
        invalidIndex = len(localPath)
        indexOutOfBoundsLatlon = getLocalWaypointLatLon(localPath=localPath, localPathIndex=invalidIndex)
        self.assertAlmostEqual(indexOutOfBoundsLatlon.lat, localPath[len(localPath)-1].lat, places=1)
        self.assertAlmostEqual(indexOutOfBoundsLatlon.lon, localPath[len(localPath)-1].lon, places=1)

        # Test index in bounds. Should give back correct latlon
        validIndex = len(localPath) // 2
        indexInBoundsLatlon = getLocalWaypointLatLon(localPath=localPath, localPathIndex=validIndex)
        self.assertAlmostEqual(indexInBoundsLatlon.lat, localPath[validIndex].lat, places=1)
        self.assertAlmostEqual(indexInBoundsLatlon.lon, localPath[validIndex].lon, places=1)

    def test_extendObstaclesArray(self):
        currentLatlon = latlon(0, 0)
        shipLatlon = XYToLatlon((1, 0), currentLatlon)
        ships = [AISShip(1000, shipLatlon.lat, shipLatlon.lon, 180, 1)]
        obstacles = extendObstaclesArray(ships, currentLatlon, 1, currentLatlon)
        self.assertFalse(isValid([0, 0], obstacles))
        self.assertFalse(isValid([1, 0], obstacles))
        self.assertTrue(isValid([2, 0], obstacles))

    def test_extendObstaclesArray2(self):
        currentLatlon = latlon(0, 0)
        shipLatlon = XYToLatlon((1, 1), currentLatlon)
        ships = [AISShip(1000, shipLatlon.lat, shipLatlon.lon, 225, sqrt(2))]
        obstacles = extendObstaclesArray(ships, currentLatlon, 1, currentLatlon)
# Uncomment below to see obstacles extended on a plot
#        ax = plt.gca()
#        for obstacle in obstacles:
#            ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))
#        plt.show()
        self.assertFalse(isValid([1, 1], obstacles))
        self.assertFalse(isValid([0, 0], obstacles))
        self.assertTrue(isValid([-1, -1], obstacles))

    def test_extendObstaclesArray3(self):
        currentLatlon = latlon(0, 0)
        shipLatlon = XYToLatlon((0, 3), currentLatlon)
        shipLatlon2 = XYToLatlon((-1, -1), currentLatlon)
        ship1 = AISShip(1000, shipLatlon.lat, shipLatlon.lon, 270, 1.5)
        ship2 = AISShip(1001, shipLatlon2.lat, shipLatlon2.lon, 45, 10)
        obstacles = extendObstaclesArray([ship1, ship2], currentLatlon, 1, currentLatlon)
# Uncomment below to see obstacles extended on a plot
#        ax = plt.gca()
#        for obstacle in obstacles:
#            ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))
#        plt.show()
        self.assertFalse(isValid([0, 0], obstacles))
        self.assertFalse(isValid([1, 1], obstacles))
        self.assertFalse(isValid([3, 3], obstacles))
        self.assertFalse(isValid([0, -1], obstacles))
        self.assertTrue(isValid([0, 4], obstacles))
        self.assertFalse(isValid([0, -1.19], obstacles))
        self.assertTrue(isValid([0, -2.3], obstacles))

    def test_localWaypointReached(self):
        refLatlon = latlon(0, 0)
        start = XYToLatlon((0, 0), refLatlon)
        waypoint = XYToLatlon((1, 1), refLatlon)
        path = [start, waypoint]
        index = 1
        sailbotPos = XYToLatlon((2, 2), refLatlon)
        sailbotPos1 = XYToLatlon((1, 0.5), refLatlon)
        self.assertTrue(localWaypointReached(sailbotPos, path, index, refLatlon))
        self.assertFalse(localWaypointReached(sailbotPos1, path, index, refLatlon))
    
    #testing cases where start and LWP have same lat or same lon
    def test_localWaypointReached_sameLat(self):
        refLatlon = latlon(0, 0)
        start = XYToLatlon((1, 1), refLatlon)
        waypoint = XYToLatlon((1, 2), refLatlon)
        path = [start, waypoint]
        index = 1
        sailbotPos = XYToLatlon((100, 2.1), refLatlon)
        sailbotPos1 = XYToLatlon((-100, 1.9), refLatlon)
        self.assertTrue(localWaypointReached(sailbotPos, path, index, refLatlon))
        self.assertFalse(localWaypointReached(sailbotPos1, path, index, refLatlon))
       
    def test_localWaypointReached_sameLon(self):
        refLatlon = latlon(0, 0)
        start = XYToLatlon((2, 1), refLatlon)
        waypoint = XYToLatlon((1, 1), refLatlon)
        path = [start, waypoint]
        index = 1
        sailbotPos = XYToLatlon((1.1, 100), refLatlon)
        sailbotPos1 = XYToLatlon((0.9, -100), refLatlon)
        self.assertFalse(localWaypointReached(sailbotPos, path, index, refLatlon))
        self.assertTrue(localWaypointReached(sailbotPos1, path, index, refLatlon))

    def test_headingToBearingDegrees(self):
        self.assertAlmostEqual(BEARING_NORTH % 360, headingToBearingDegrees(HEADING_NORTH) % 360, places=3)
        self.assertAlmostEqual(BEARING_SOUTH % 360, headingToBearingDegrees(HEADING_SOUTH) % 360, places=3)
        self.assertAlmostEqual(BEARING_EAST % 360, headingToBearingDegrees(HEADING_EAST) % 360, places=3)

        bearingDirection = (2*BEARING_SOUTH + 1*BEARING_WEST) / 3
        headingDirection = (2*HEADING_SOUTH + 1*HEADING_WEST) / 3
        self.assertAlmostEqual(bearingDirection % 360, headingToBearingDegrees(headingDirection) % 360, places=3)

    def test_measuredWindToGlobalWind_basic(self):
        # (Boat moving + no measured wind) => (global wind velocity == boat velocity)
        globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(measuredWindSpeed=0, measuredWindDirectionDegrees=BOAT_RIGHT, boatSpeed=1, headingDegrees=HEADING_NORTH)
        self.assertAlmostEqual(1, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(HEADING_NORTH, globalWindDirectionDegrees, places=3)

        # (Boat not moving + measured wind) => (global wind speed == measured wind speed) + (global wind dir == measured wind dir - 90) b/c 0 measured direction = right of boat which is -90 in global when boat is pointed east)
        measuredDirection = (2*BOAT_FORWARD + 1*BOAT_RIGHT) / 3 # 60 degrees
        globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(measuredWindSpeed=1.2, measuredWindDirectionDegrees=measuredDirection, boatSpeed=0, headingDegrees=HEADING_EAST)
        self.assertAlmostEqual(1.2, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(measuredDirection-90, globalWindDirectionDegrees, places=3)

        # (Boat and measured wind along the same axis) => (global wind velocity == boat + measured wind velocity)
        globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(measuredWindSpeed=5, measuredWindDirectionDegrees=BOAT_FORWARD, boatSpeed=7, headingDegrees=90)
        self.assertAlmostEqual(12, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(90, globalWindDirectionDegrees, places=3)

    def test_measuredWindToGlobalWind_advanced(self):
        # Setup test parameters
        headingDegrees = 20
        boatSpeedKmph = 4
        measuredWindDirectionDegrees = 10
        measuredWindSpeedKmph = 2
        globalWindSpeedKmph, globalWindDirectionDegrees = measuredWindToGlobalWind(measuredWindSpeedKmph, measuredWindDirectionDegrees, boatSpeedKmph, headingDegrees)
        self.assertAlmostEqual(4.7726691528, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(-4.373700424, globalWindDirectionDegrees, places=3)

    def test_globalWindToMeasuredWind_basic(self):
        # (Boat moving + no global wind) => (measured wind speed == boat speed) + (measured wind dir == -boat dir == -BOAT_FORWARD)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=0, globalWindDirectionDegrees=HEADING_EAST, boatSpeed=2, headingDegrees=HEADING_EAST)
        self.assertAlmostEqual(2, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual((BOAT_FORWARD+180) % 360, measuredWindDirectionDegrees % 360, places=3)

        # (Boat not moving + global wind) => (measured wind speed == global wind speed) + (measured wind dir == global wind dir + 90) b/c 0 degrees global is 90 degrees for measured wind, since boat is pointing EAST)
        globalDirection = (2*HEADING_NORTH + 1*HEADING_EAST) / 3 # 60 degrees
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=globalDirection, boatSpeed=0, headingDegrees=HEADING_EAST)
        self.assertAlmostEqual(10, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual((globalDirection+90) % 360, measuredWindDirectionDegrees % 360, places=3)

        # (Boat and measured wind along the same axis, with boat speed > global wind speed) => (measured wind == global wind - boat speed) + (measured wind dir along boat forward)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=HEADING_SOUTH, boatSpeed=3, headingDegrees=HEADING_SOUTH)
        self.assertAlmostEqual(10 - 3, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual(BOAT_FORWARD, measuredWindDirectionDegrees % 360, places=3)

    def test_globalWindToMeasuredWind_advanced(self):
        # Setup test parameters
        headingDegrees = 120
        boatSpeedKmph = 14
        globalWindDirectionDegrees = 12
        globalWindSpeedKmph = 11
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeedKmph, globalWindDirectionDegrees, boatSpeedKmph, headingDegrees)

        self.assertAlmostEqual(20.30214851358062, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual(-58.98273894650611, measuredWindDirectionDegrees, places=3)

    def test_globalWindToMeasuredWind_and_measuredWindToGlobalWind(self):
        # Setup test parameters
        headingDegrees = 150
        boatSpeedKmph = 14
        globalWindDirectionDegrees = 78
        globalWindSpeedKmph = 13
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeedKmph, globalWindDirectionDegrees, boatSpeedKmph, headingDegrees)
        calculatedGlobalWindSpeedKmph, calculatedGlobalWindDirectionDegrees = measuredWindToGlobalWind(measuredWindSpeedKmph, measuredWindDirectionDegrees, boatSpeedKmph, headingDegrees)

        # Test that we get back the same global wind as we started with
        self.assertAlmostEqual(calculatedGlobalWindSpeedKmph, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(calculatedGlobalWindDirectionDegrees, globalWindDirectionDegrees, places=3)

    def test_obstacleOnPath(self):
        # Create simple path
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=90, boatSpeed=0, headingDegrees=0)
        state = BoatState(globalWaypoint=latlon(1,1), position=latlon(0,0), measuredWindDirectionDegrees=measuredWindDirectionDegrees, measuredWindSpeedKmph=measuredWindSpeedKmph, AISData=AISMsg(), headingDegrees=0, speedKmph=0)
        localPathSS, referenceLatlon = createLocalPathSS(state)

        # Sanity check with no obstacles
        self.assertFalse(obstacleOnPath(state, localPathSS, referenceLatlon))

        # Put obstacle on path
        waypoint0 = localPathSS.getSolutionPath().getState(0)
        waypoint1 = localPathSS.getSolutionPath().getState(1)
        between = [waypoint0.getX() + (waypoint1.getX() - waypoint0.getX()) / 2, waypoint0.getY() + (waypoint1.getY() - waypoint0.getY()) / 2]
        shipLatlon = XYToLatlon(between, referenceLatlon)
        state.AISData = AISMsg([AISShip(0, shipLatlon.lat, shipLatlon.lon, 0, 1000)])

        # Sanity check has obstacle on path
        waypoint0Latlon = XYToLatlon([waypoint0.getX(), waypoint0.getY()], referenceLatlon)
        waypoint1Latlon = XYToLatlon([waypoint1.getX(), waypoint1.getY()], referenceLatlon)
        self.assertTrue(obstacleOnPath(state, localPathSS, referenceLatlon), msg='{}, {}, {}'.format(waypoint0Latlon, waypoint1Latlon, shipLatlon))


if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_utilities', TestUtilities)
