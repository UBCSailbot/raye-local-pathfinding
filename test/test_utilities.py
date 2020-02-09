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

        # Setup latlon and bearing
        northBearing = 0
        position = latlon(50, -120)

        # Setup destination
        destination = distance(kilometers=1).destination(point=(position.lat, position.lon), bearing=northBearing)
        localWaypoint = latlon(destination.latitude, destination.longitude)

        # Test desiredHeading
        desiredHeading = getDesiredHeading(position=position, localWaypoint=localWaypoint)
        self.assertAlmostEqual(desiredHeading, -northBearing+90, places=1)

    def test_getDesiredHeading_east(self):
        # Setup latlon and bearing
        eastBearing = 90
        position = latlon(50, -120)

        # Setup destination
        destination = distance(kilometers=1).destination(point=(position.lat, position.lon), bearing=eastBearing)
        localWaypoint = latlon(destination.latitude, destination.longitude)

        # Test desiredHeading
        desiredHeading = getDesiredHeading(position=position, localWaypoint=localWaypoint)
        self.assertAlmostEqual(desiredHeading, -eastBearing+90, places=1)

    def test_globalWaypointReached(self):
        position = latlon(35, -150)

        # Far away globalWaypoint unreached
        unreached = distance(kilometers=500).destination(point=(position.lat, position.lon), bearing=90)
        unreachedGlobalWaypoint = latlon(unreached.latitude, unreached.longitude)
        self.assertFalse(globalWaypointReached(position, unreachedGlobalWaypoint))

        # Far away globalWaypoint reached
        reached = distance(kilometers=50).destination(point=(position.lat, position.lon), bearing=90)
        reachedGlobalWaypoint = latlon(reached.latitude, reached.longitude)
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

    #helper method for extendObstaclesArray to check if position is valid        
    def isValid(self, x, y, obstacles):
        for obstacle in obstacles:
            if sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2)) - obstacle.radius <= 0:
                return False
        return True
        
    def test_extendObstaclesArray(self):
        currentLatlon = latlon(0, 0)
        shipLatlon = XYToLatlon((1, 0), currentLatlon)
        ships = [AIS_ship(1000, shipLatlon.lat, shipLatlon.lon, 180, 1)]
        obstacles = extendObstaclesArray(ships, currentLatlon)
        self.assertFalse(self.isValid(0, 0, obstacles))
        self.assertFalse(self.isValid(1, 0, obstacles))
        self.assertTrue(self.isValid(2, 0, obstacles)) 

    def test_extendObstaclesArray2(self):
        currentLatlon = latlon(0, 0)
        shipLatlon = XYToLatlon((1, 1), currentLatlon)
        ships = [AIS_ship(1000, shipLatlon.lat, shipLatlon.lon, 225, sqrt(2))]
        obstacles = extendObstaclesArray(ships, currentLatlon)
#        ax = plt.gca()
#        for obstacle in obstacles:
#            ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))
#        plt.show()
        self.assertFalse(self.isValid(1, 1, obstacles))
        self.assertFalse(self.isValid(0, 0, obstacles))
        self.assertTrue(self.isValid(-1, -1, obstacles))

    def test_extendObstaclesArray3(self):
        currentLatlon = latlon(0, 0)
        shipLatlon = XYToLatlon((0, 3), currentLatlon)
        shipLatlon2 = XYToLatlon((-1, -1), currentLatlon)
        ship1 = AIS_ship(1000, shipLatlon.lat, shipLatlon.lon, 270, 4)
        ship2 = AIS_ship(1001, shipLatlon2.lat, shipLatlon2.lon, 45, 10)
        obstacles = extendObstaclesArray([ship1, ship2], currentLatlon)
        ax = plt.gca()
        for obstacle in obstacles:
            ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))
        plt.show()
        self.assertFalse(self.isValid(0, 0, obstacles))
        self.assertFalse(self.isValid(1, 1, obstacles))
        self.assertFalse(self.isValid(3, 3, obstacles))
        self.assertFalse(self.isValid(0, -1, obstacles))
        self.assertTrue(self.isValid(0, 4, obstacles))

if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_utilities', TestUtilities)
