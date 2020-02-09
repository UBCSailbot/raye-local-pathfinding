#! /usr/bin/env python

# Add python directory to path
import sys, os
testdir = os.path.dirname(__file__)
srcdir = '../python'
sys.path.insert(0, os.path.abspath(os.path.join(testdir, srcdir)))

import unittest
import rostest
from local_pathfinding.msg import latlon, AIS_ship
from utilities import *
from geopy.distance import distance

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


if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_utilities', TestUtilities)
