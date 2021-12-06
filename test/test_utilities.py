#!/usr/bin/env python

import local_imports  # Must be first import, as it adds python directory to path
from geopy.distance import distance
import utilities as utils
from planner_helpers import abs_angle_dist_degrees
from sailbot_msg.msg import latlon
import rostest
import unittest


# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class TestUtilities(unittest.TestCase):
    def test_latlonToXY_and_XYToLatlon_advanced(self):
        # Setup latlons
        startLatlon = latlon(49.263022, -123.023447)
        endLatlon = latlon(47.7984, -125.3319)

        # Test latlonToXY
        x, y = utils.latlonToXY(latlon=endLatlon, referenceLatlon=startLatlon)
        # endLatlon is about 168.0158959716741km west of startLatlon
        self.assertAlmostEqual(x, -168.0158959716741, places=2)
        # endLatLon is about 162.8668880228988km south of startLatlon
        self.assertAlmostEqual(y, -162.8668880228988, places=2)

        # Test XYToLatlon
        calculatedEndLatlon = utils.XYToLatlon(xy=[x, y], referenceLatlon=startLatlon)
        # calculatedEndLatlon should be same as endLatlon
        self.assertAlmostEqual(calculatedEndLatlon.lat, endLatlon.lat, places=1)
        self.assertAlmostEqual(calculatedEndLatlon.lon, endLatlon.lon, places=1)

    def test_latlonToXY_and_XYToLatlon_basic(self):
        # Setup latlon
        testLatlon = latlon(50, -120)

        # Test latlonToXY
        x, y = utils.latlonToXY(latlon=testLatlon, referenceLatlon=testLatlon)
        self.assertAlmostEqual(x, 0, places=2)  # xy=(0,0) at reference point
        self.assertAlmostEqual(y, 0, places=2)

        # Test XYToLatlon
        calculatedLatlon = utils.XYToLatlon(xy=[x, y], referenceLatlon=testLatlon)
        # calculatedLatlon should be same as testLatlon
        self.assertAlmostEqual(calculatedLatlon.lat, testLatlon.lat, places=1)
        self.assertAlmostEqual(calculatedLatlon.lon, testLatlon.lon, places=1)

    def test_getDesiredHeadingDegrees_north(self):
        # Setup latlon
        position = latlon(50, -120)

        # Setup destination
        northDestination = distance(kilometers=1).destination(point=(position.lat, position.lon),
                                                              bearing=utils.BEARING_NORTH)
        northDestination = latlon(northDestination.latitude, northDestination.longitude)

        # Test desiredHeading
        desiredHeading = utils.getDesiredHeadingDegrees(position=position, localWaypoint=northDestination)
        self.assertAlmostEqual(desiredHeading, utils.HEADING_NORTH, places=1)

    def test_getDesiredHeadingDegrees_east(self):
        # Setup latlon
        position = latlon(50, -120)

        # Setup destination
        eastDestination = distance(kilometers=1).destination(point=(position.lat, position.lon),
                                                             bearing=utils.BEARING_EAST)
        eastDestination = latlon(eastDestination.latitude, eastDestination.longitude)

        # Test desiredHeading
        desiredHeading = utils.getDesiredHeadingDegrees(position=position, localWaypoint=eastDestination)
        self.assertAlmostEqual(desiredHeading, utils.HEADING_EAST, places=1)

    def test_bearingToHeadingDegrees(self):
        # Basic tests
        self.assertAlmostEqual(abs_angle_dist_degrees(utils.bearingToHeadingDegrees(utils.BEARING_NORTH),
                                                      utils.HEADING_NORTH),
                               0,
                               places=3)
        self.assertAlmostEqual(abs_angle_dist_degrees(utils.bearingToHeadingDegrees(utils.BEARING_SOUTH),
                                                      utils.HEADING_SOUTH),
                               0,
                               places=3)
        self.assertAlmostEqual(abs_angle_dist_degrees(utils.bearingToHeadingDegrees(utils.BEARING_EAST),
                                                      utils.HEADING_EAST),
                               0,
                               places=3)

        # Advanced test
        bearingDirection = (2 * utils.BEARING_SOUTH + 1 * utils.BEARING_WEST) / 3
        headingDirection = (2 * utils.HEADING_SOUTH + 1 * utils.HEADING_WEST) / 3
        self.assertAlmostEqual(abs_angle_dist_degrees(utils.bearingToHeadingDegrees(bearingDirection),
                                                      headingDirection),
                               0,
                               places=3)

    def test_headingToBearingDegrees(self):
        # Basic tests
        self.assertAlmostEqual(abs_angle_dist_degrees(utils.BEARING_NORTH,
                                                      utils.headingToBearingDegrees(utils.HEADING_NORTH)),
                               0,
                               places=3)
        self.assertAlmostEqual(abs_angle_dist_degrees(utils.BEARING_SOUTH,
                                                      utils.headingToBearingDegrees(utils.HEADING_SOUTH)),
                               0,
                               places=3)
        self.assertAlmostEqual(abs_angle_dist_degrees(utils.BEARING_EAST,
                                                      utils.headingToBearingDegrees(utils.HEADING_EAST)),
                               0,
                               places=3)

        # Advanced test
        bearingDirection = (2 * utils.BEARING_SOUTH + 1 * utils.BEARING_WEST) / 3
        headingDirection = (2 * utils.HEADING_SOUTH + 1 * utils.HEADING_WEST) / 3
        self.assertAlmostEqual(abs_angle_dist_degrees(bearingDirection,
                                                      utils.headingToBearingDegrees(headingDirection)),
                               0,
                               places=3)

    def test_measuredWindToGlobalWind_basic(self):
        # (Boat moving + no measured wind) => (global wind velocity == boat velocity)
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            measuredWindSpeed=0, measuredWindDirectionDegrees=utils.BOAT_RIGHT, boatSpeed=1,
            headingDegrees=utils.HEADING_NORTH, trackMadeGoodDegrees=utils.HEADING_NORTH)
        self.assertAlmostEqual(1, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(utils.HEADING_NORTH, globalWindDirectionDegrees, places=3)

        # (Boat not moving + measured wind) => (global wind speed == measured wind speed)
        #                                      + (global wind dir == measured wind dir - 90)
        # b/c 0 measured direction = right of boat which is -90 in global when boat is pointed east)
        measuredDirection = (2 * utils.BOAT_FORWARD + 1 * utils.BOAT_RIGHT) / 3  # 60 degrees
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            measuredWindSpeed=1.2, measuredWindDirectionDegrees=measuredDirection, boatSpeed=0,
            headingDegrees=utils.HEADING_EAST, trackMadeGoodDegrees=utils.HEADING_EAST)
        self.assertAlmostEqual(1.2, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(measuredDirection - 90, globalWindDirectionDegrees, places=3)

        # (Boat and measured wind along the same axis) => (global wind velocity == boat + measured wind velocity)
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            measuredWindSpeed=5, measuredWindDirectionDegrees=utils.BOAT_FORWARD, boatSpeed=7, headingDegrees=90,
            trackMadeGoodDegrees=90)
        self.assertAlmostEqual(12, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(90, globalWindDirectionDegrees, places=3)

        # (Boat moving backwards, measured wind is opposite of heading) =>
        # (global wind velocity == boat + measured wind velocity, global wind is same direction as boat movement)
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            measuredWindSpeed=5, measuredWindDirectionDegrees=utils.BOAT_BACKWARD, boatSpeed=5,
            headingDegrees=utils.HEADING_EAST, trackMadeGoodDegrees=utils.HEADING_WEST)
        self.assertAlmostEqual(10, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(utils.HEADING_WEST, globalWindDirectionDegrees, places=3)

    def test_measuredWindToGlobalWind_advanced(self):
        # Setup test parameters
        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            measuredWindSpeed=2, measuredWindDirectionDegrees=10, boatSpeed=4, headingDegrees=20,
            trackMadeGoodDegrees=20)
        self.assertAlmostEqual(4.7726691528, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(-4.373700424, globalWindDirectionDegrees, places=3)

        globalWindSpeedKmph, globalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            measuredWindSpeed=1, measuredWindDirectionDegrees=60, boatSpeed=10, headingDegrees=45,
            trackMadeGoodDegrees=40)
        self.assertAlmostEqual(10.914492921832556, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(37.78090491698471, globalWindDirectionDegrees, places=3)

    def test_globalWindToMeasuredWind_basic(self):
        # (Boat moving + no global wind) => (measured wind speed == boat speed)
        #                                   + (measured wind dir == -boat dir == -utils.BOAT_FORWARD)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=0, globalWindDirectionDegrees=utils.HEADING_EAST, boatSpeed=2,
            headingDegrees=utils.HEADING_EAST, trackMadeGoodDegrees=utils.HEADING_EAST)
        self.assertAlmostEqual(2, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual((utils.BOAT_FORWARD + 180) % 360, measuredWindDirectionDegrees % 360, places=3)

        # (Boat not moving + global wind) => (measured wind speed == global wind speed)
        #                                    + (measured wind dir == global wind dir + 90) b/c 0 degrees global is
        #                                       90 degrees for measured wind, since boat is pointing EAST)
        globalDirection = (2 * utils.HEADING_NORTH + 1 * utils.HEADING_EAST) / 3  # 60 degrees
        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=globalDirection, boatSpeed=0,
            headingDegrees=utils.HEADING_EAST, trackMadeGoodDegrees=utils.HEADING_EAST)
        self.assertAlmostEqual(10, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual((globalDirection + 90) % 360, measuredWindDirectionDegrees % 360, places=3)

        # (Boat and measured wind along the same axis, with boat speed > global wind speed) =>
        # (measured wind == global wind - boat speed) + (measured wind dir along boat forward)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=utils.HEADING_SOUTH, boatSpeed=3,
            headingDegrees=utils.HEADING_SOUTH, trackMadeGoodDegrees=utils.HEADING_SOUTH)
        self.assertAlmostEqual(10 - 3, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual(utils.BOAT_FORWARD, measuredWindDirectionDegrees % 360, places=3)

        # (Boat moving backwards, global wind in same direction) =>
        # (measured wind velocity = global wind velocity - boat, measured wind direction = opposite of heading)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=utils.HEADING_EAST, boatSpeed=5,
            headingDegrees=utils.HEADING_WEST, trackMadeGoodDegrees=utils.HEADING_EAST)
        self.assertAlmostEqual(5, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual(-90, measuredWindDirectionDegrees, places=3)

    def test_globalWindToMeasuredWind_advanced(self):
        # Setup test parameters
        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=11, globalWindDirectionDegrees=12, boatSpeed=14,
            headingDegrees=120, trackMadeGoodDegrees=120)
        self.assertAlmostEqual(20.30214851358062, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual(-58.98273894650611, measuredWindDirectionDegrees, places=3)

        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=5, globalWindDirectionDegrees=12, boatSpeed=14,
            headingDegrees=30, trackMadeGoodDegrees=210)
        self.assertAlmostEqual(18.818818036245567, measuredWindSpeedKmph, places=3)
        self.assertAlmostEqual(85.2905326169815, measuredWindDirectionDegrees, places=3)

    def test_globalWindToMeasuredWind_and_measuredWindToGlobalWind(self):
        # Setup test parameters
        headingDegrees = 150
        trackMadeGoodDegrees = 100
        boatSpeedKmph = 14
        globalWindDirectionDegrees = 78
        globalWindSpeedKmph = 13
        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=globalWindSpeedKmph, globalWindDirectionDegrees=globalWindDirectionDegrees,
            boatSpeed=boatSpeedKmph, headingDegrees=headingDegrees, trackMadeGoodDegrees=trackMadeGoodDegrees)
        calculatedGlobalWindSpeedKmph, calculatedGlobalWindDirectionDegrees = utils.measuredWindToGlobalWind(
            measuredWindSpeedKmph, measuredWindDirectionDegrees, boatSpeedKmph, headingDegrees,
            trackMadeGoodDegrees=trackMadeGoodDegrees)

        # Test that we get back the same global wind as we started with
        self.assertAlmostEqual(calculatedGlobalWindSpeedKmph, globalWindSpeedKmph, places=3)
        self.assertAlmostEqual(calculatedGlobalWindDirectionDegrees, globalWindDirectionDegrees, places=3)


if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_utilities', TestUtilities)
