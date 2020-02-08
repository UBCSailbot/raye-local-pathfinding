#! /usr/bin/env python

import sys, os
testdir = os.path.dirname(__file__)
srcdir = '../python'
sys.path.insert(0, os.path.abspath(os.path.join(testdir, srcdir)))

import unittest
import rostest
from local_pathfinding.msg import latlon, AIS_ship
from utilities import *

threshold = 0.001
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

if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_utilities', TestUtilities)
