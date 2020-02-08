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

        # Setup latlon and bearing
        northBearing = 0
        position = latlon(50, -120)

        # Setup destination
        positionToLocalWaypointDistance = distance(kilometers=1)
        destination = positionToLocalWaypointDistance.destination(point=(position.lat, position.lon), bearing=northBearing)
        localWaypoint = latlon(destination.latitude, destination.longitude)

        # Test desiredHeading
        desiredHeading = getDesiredHeading(position=position, localWaypoint=localWaypoint)
        self.assertAlmostEqual(desiredHeading, -northBearing+90, places=1)

    def test_getDesiredHeading_east(self):
        # Setup latlon and bearing
        eastBearing = 90
        position = latlon(50, -120)

        # Setup destination
        positionToLocalWaypointDistance = distance(kilometers=1)
        destination = positionToLocalWaypointDistance.destination(point=(position.lat, position.lon), bearing=eastBearing)
        localWaypoint = latlon(destination.latitude, destination.longitude)

        # Test desiredHeading
        desiredHeading = getDesiredHeading(position=position, localWaypoint=localWaypoint)
        self.assertAlmostEqual(desiredHeading, -eastBearing+90, places=1)


if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_utilities', TestUtilities)
