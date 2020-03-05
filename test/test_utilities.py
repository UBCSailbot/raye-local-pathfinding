#! /usr/bin/env python

# Add python directory to path
import sys, os
testdir = os.path.dirname(__file__)
srcdir = '../python'
sys.path.insert(0, os.path.abspath(os.path.join(testdir, srcdir)))

import unittest
import rostest
from local_pathfinding.msg import latlon, AISShip, AISMsg
from planner_helpers import UPWIND_MAX_ANGLE_DEGREES, DOWNWIND_MAX_ANGLE_DEGREES
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
        x, y = latlonToXY(latlon=endLatlon, referenceLatlon=startLatlon)
        self.assertAlmostEqual(x, -168.0158959716741, places=2)  # endLatlon is about 168.0158959716741km west of startLatlon
        self.assertAlmostEqual(y, -162.8668880228988, places=2)  # endLatLon is about 162.8668880228988km south of startLatlon

        # Test XYToLatlon
        calculatedEndLatlon = XYToLatlon(xy=[x, y], referenceLatlon=startLatlon)
        self.assertAlmostEqual(calculatedEndLatlon.lat, endLatlon.lat, places=1)  # calculatedEndLatlon should be same as endLatlon
        self.assertAlmostEqual(calculatedEndLatlon.lon, endLatlon.lon, places=1)

    def test_latlonToXY_and_XYToLatlon_basic(self):
        # Setup latlon
        testLatlon = latlon(50, -120)

        # Test latlonToXY
        x, y = latlonToXY(latlon=testLatlon, referenceLatlon=testLatlon)
        self.assertAlmostEqual(x, 0, places=2)  # xy=(0,0) at reference point
        self.assertAlmostEqual(y, 0, places=2)

        # Test XYToLatlon
        calculatedLatlon = XYToLatlon(xy=[x, y], referenceLatlon=testLatlon)
        self.assertAlmostEqual(calculatedLatlon.lat, testLatlon.lat, places=1)  # calculatedLatlon should be same as testLatlon
        self.assertAlmostEqual(calculatedLatlon.lon, testLatlon.lon, places=1)

    def test_getDesiredHeading_north(self):
        # Setup latlon
        position = latlon(50, -120)

        # Setup destination
        northDestination = distance(kilometers=1).destination(point=(position.lat, position.lon), bearing=BEARING_NORTH)
        northDestination = latlon(northDestination.latitude, northDestination.longitude)

        # Test desiredHeading
        desiredHeading = getDesiredHeading(position=position, localWaypoint=northDestination)
        self.assertAlmostEqual(desiredHeading, HEADING_NORTH, places=1)

    def test_getDesiredHeading_east(self):
        # Setup latlon
        position = latlon(50, -120)

        # Setup destination
        eastDestination = distance(kilometers=1).destination(point=(position.lat, position.lon), bearing=BEARING_EAST)
        eastDestination = latlon(eastDestination.latitude, eastDestination.longitude)

        # Test desiredHeading
        desiredHeading = getDesiredHeading(position=position, localWaypoint=eastDestination)
        self.assertAlmostEqual(desiredHeading, HEADING_EAST, places=1)

    def test_globalWaypointReached(self):
        # Setup latlon
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

        # Create test localPath
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
        # Setup starting at (0,0) with obstacle at (1,0) heading west
        currentLatlon = latlon(0, 0)
        referenceLatlon = currentLatlon
        shipLatlon = XYToLatlon(xy=(1, 0), referenceLatlon=referenceLatlon)
        ships = [AISShip(ID=1000, lat=shipLatlon.lat, lon=shipLatlon.lon, headingDegrees=HEADING_WEST, speedKmph=1)]
        obstacles = extendObstaclesArray(aisArray=ships, sailbotPosition=currentLatlon, sailbotSpeedKmph=1, referenceLatlon=referenceLatlon)

        self.assertFalse(isValid(xy=[0, 0], obstacles=obstacles))
        self.assertFalse(isValid(xy=[1, 0], obstacles=obstacles))
        self.assertTrue(isValid(xy=[2, 0], obstacles=obstacles))

    def test_extendObstaclesArray2(self):
        # Setup starting at (0,0) with obstacle at (1,1) heading south-west
        currentLatlon = latlon(0, 0)
        referenceLatlon = currentLatlon
        shipLatlon = XYToLatlon(xy=(1, 1), referenceLatlon=referenceLatlon)
        ships = [AISShip(ID=1000, lat=shipLatlon.lat, lon=shipLatlon.lon, headingDegrees=225, speedKmph=sqrt(2))]
        obstacles = extendObstaclesArray(aisArray=ships, sailbotPosition=currentLatlon, sailbotSpeedKmph=1, referenceLatlon=referenceLatlon)
# Uncomment below to see obstacles extended on a plot
#        ax = plt.gca()
#        for obstacle in obstacles:
#            ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))
#        plt.show()
        self.assertFalse(isValid(xy=[1, 1], obstacles=obstacles))
        self.assertFalse(isValid(xy=[0, 0], obstacles=obstacles))
        self.assertTrue(isValid(xy=[-1, -1], obstacles=obstacles))

    def test_extendObstaclesArray3(self):
        # Setup starting at (0,0) with obstacles at (0,3) and (-1,-1)
        currentLatlon = latlon(0, 0)
        referenceLatlon = currentLatlon
        shipLatlon = XYToLatlon(xy=(0, 3), referenceLatlon=referenceLatlon)
        shipLatlon2 = XYToLatlon(xy=(-1, -1), referenceLatlon=referenceLatlon)
        ship1 = AISShip(ID=1000, lat=shipLatlon.lat, lon=shipLatlon.lon, headingDegrees=270, speedKmph=1.5)
        ship2 = AISShip(ID=1001, lat=shipLatlon2.lat, lon=shipLatlon2.lon, headingDegrees=45, speedKmph=10)
        obstacles = extendObstaclesArray(aisArray=[ship1, ship2], sailbotPosition=currentLatlon, sailbotSpeedKmph=1, referenceLatlon=referenceLatlon)
# Uncomment below to see obstacles extended on a plot
#        ax = plt.gca()
#        for obstacle in obstacles:
#            ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))
#        plt.show()
        self.assertFalse(isValid(xy=[0, 0], obstacles=obstacles))
        self.assertFalse(isValid(xy=[1, 1], obstacles=obstacles))
        self.assertFalse(isValid(xy=[3, 3], obstacles=obstacles))
        self.assertFalse(isValid(xy=[0, -1], obstacles=obstacles))
        self.assertTrue(isValid(xy=[0, 4], obstacles=obstacles))
        self.assertFalse(isValid(xy=[0, -1.19], obstacles=obstacles))
        self.assertTrue(isValid(xy=[0, -2.3], obstacles=obstacles))

    def test_localWaypointReached(self):
        # Setup path from (0,0) to (1,1)
        refLatlon = latlon(0, 0)
        start = XYToLatlon(xy=(0, 0), referenceLatlon=refLatlon)
        waypoint = XYToLatlon(xy=(1, 1), referenceLatlon=refLatlon)
        path = [start, waypoint]
        index = 1

        # Test reachedPos
        reachedPos = XYToLatlon(xy=(2, 2), referenceLatlon=refLatlon)
        self.assertTrue(localWaypointReached(position=reachedPos, localPath=path, localPathIndex=index, refLatlon=refLatlon))

        # Test notReachedPos
        notReachedPos = XYToLatlon(xy=(1, 0.5), referenceLatlon=refLatlon)
        self.assertFalse(localWaypointReached(position=notReachedPos, localPath=path, localPathIndex=index, refLatlon=refLatlon))
    
    #testing cases where start and LWP have same lat or same lon
    def test_localWaypointReached_sameLat(self):
        # Setup path from (1,1) to (1,2)
        refLatlon = latlon(0, 0)
        start = XYToLatlon(xy=(1, 1), referenceLatlon=refLatlon)
        waypoint = XYToLatlon(xy=(1, 2), referenceLatlon=refLatlon)
        path = [start, waypoint]
        index = 1

        # Test reachedPos
        reachedPos = XYToLatlon(xy=(100, 2.1), referenceLatlon=refLatlon)
        self.assertTrue(localWaypointReached(position=reachedPos, localPath=path, localPathIndex=index, refLatlon=refLatlon))

        # Test notReachedPos
        notReachedPos = XYToLatlon(xy=(-100, 1.9), referenceLatlon=refLatlon)
        self.assertFalse(localWaypointReached(position=notReachedPos, localPath=path, localPathIndex=index, refLatlon=refLatlon))

    def test_localWaypointReached_sameLon(self):
        # Setup path from (2,1) to (1,1)
        refLatlon = latlon(0, 0)
        start = XYToLatlon(xy=(2, 1), referenceLatlon=refLatlon)
        waypoint = XYToLatlon(xy=(1, 1), referenceLatlon=refLatlon)
        path = [start, waypoint]
        index = 1

        # Test reachedPos
        reachedPos = XYToLatlon(xy=(0.9, -100), referenceLatlon=refLatlon)
        self.assertTrue(localWaypointReached(position=reachedPos, localPath=path, localPathIndex=index, refLatlon=refLatlon))

        # Test notReachedPos
        notReachedPos = XYToLatlon(xy=(1.1, 100), referenceLatlon=refLatlon)
        self.assertFalse(localWaypointReached(position=notReachedPos, localPath=path, localPathIndex=index, refLatlon=refLatlon))

    def test_headingToBearingDegrees(self):
        # Basic tests
        self.assertAlmostEqual(BEARING_NORTH % 360, headingToBearingDegrees(HEADING_NORTH) % 360, places=3)
        self.assertAlmostEqual(BEARING_SOUTH % 360, headingToBearingDegrees(HEADING_SOUTH) % 360, places=3)
        self.assertAlmostEqual(BEARING_EAST % 360, headingToBearingDegrees(HEADING_EAST) % 360, places=3)

        # Advanced test
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

    def test_createLocalPathSS(self):
        # Create path that require tacking (wind and nextGlobalWaypoint are 45 degrees)
        start = latlon(0, 0)
        goal = latlon(2, 2)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=45, boatSpeed=0, headingDegrees=0)
        state = BoatState(globalWaypoint=goal, position=start, measuredWindDirectionDegrees=measuredWindDirectionDegrees, measuredWindSpeedKmph=measuredWindSpeedKmph, AISData=AISMsg(), headingDegrees=0, speedKmph=0)
        localPathSS, referenceLatlon = createLocalPathSS(state, runtimeSeconds=1, numRuns=3)
        solutionPath = localPathSS.getSolutionPath()

        # Check that first and last states match the setup start and goal
        firstState = solutionPath.getState(0)
        startState = latlonToXY(latlon=start, referenceLatlon=referenceLatlon)
        self.assertAlmostEqual(startState[0], firstState.getX(), places=3)
        self.assertAlmostEqual(startState[1], firstState.getY(), places=3)

        lastState = solutionPath.getState(len(solutionPath.getStates()) - 1)
        goalState = latlonToXY(latlon=goal, referenceLatlon=referenceLatlon)
        self.assertAlmostEqual(goalState[0], lastState.getX(), places=3)
        self.assertAlmostEqual(goalState[1], lastState.getY(), places=3)

        # Check that the distance between waypoints is correct
        for stateIndex in range(1, len(solutionPath.getStates())):
            prevState = solutionPath.getState(stateIndex - 1)
            currState = solutionPath.getState(stateIndex)
            distance = ((currState.getX() - prevState.getX())**2 + (currState.getY() - prevState.getY())**2)**0.5
            self.assertTrue(AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM / 2 < distance and distance < 2 * AVG_DISTANCE_BETWEEN_LOCAL_WAYPOINTS_KM)

    def test_upwindOrDownwindOnPath(self):
        # Create simple path from latlon(0,0) to latlon(1,1)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=90, boatSpeed=0, headingDegrees=0)
        state = BoatState(globalWaypoint=latlon(1,1), position=latlon(0,0), measuredWindDirectionDegrees=measuredWindDirectionDegrees, measuredWindSpeedKmph=measuredWindSpeedKmph, AISData=AISMsg(), headingDegrees=0, speedKmph=0)
        localPathSS, referenceLatlon = createLocalPathSS(state, runtimeSeconds=3, numRuns=1)
        solutionPath = localPathSS.getSolutionPath()

        # Set state with global wind direction nearly same as boat current direction (sailing downwind)
        nextLocalWaypointIndex = 1
        boatPositionXY = latlonToXY(state.position, referenceLatlon)
        desiredHeadingDegrees = math.degrees(math.atan2(solutionPath.getState(nextLocalWaypointIndex).getY() - boatPositionXY[1], solutionPath.getState(nextLocalWaypointIndex).getX() - boatPositionXY[0]))
        downwindGlobalWindDirectionDegrees = desiredHeadingDegrees + DOWNWIND_MAX_ANGLE_DEGREES / 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=downwindGlobalWindDirectionDegrees, boatSpeed=1, headingDegrees=120)
        downwindState = BoatState(globalWaypoint=latlon(1,1), position=latlon(0,0), measuredWindDirectionDegrees=measuredWindDirectionDegrees, measuredWindSpeedKmph=measuredWindSpeedKmph, AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        self.assertTrue(upwindOrDownwindOnPath(state=downwindState, nextLocalWaypointIndex=nextLocalWaypointIndex, localPathSS=localPathSS, referenceLatlon=referenceLatlon, numLookAheadWaypoints=1))

        # Set state with global wind direction nearly 180 degrees from boat current direction (sailing upwind)
        nextLocalWaypointIndex = 1
        boatPositionXY = latlonToXY(state.position, referenceLatlon)
        desiredHeadingDegrees = math.degrees(math.atan2(solutionPath.getState(nextLocalWaypointIndex).getY() - boatPositionXY[1], solutionPath.getState(nextLocalWaypointIndex).getX() - boatPositionXY[0]))
        upwindGlobalWindDirectionDegrees = desiredHeadingDegrees + 180 + UPWIND_MAX_ANGLE_DEGREES / 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=upwindGlobalWindDirectionDegrees, boatSpeed=1, headingDegrees=120)
        upwindState = BoatState(globalWaypoint=latlon(1,1), position=latlon(0,0), measuredWindDirectionDegrees=measuredWindDirectionDegrees, measuredWindSpeedKmph=measuredWindSpeedKmph, AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        self.assertTrue(upwindOrDownwindOnPath(state=upwindState, nextLocalWaypointIndex=nextLocalWaypointIndex, localPathSS=localPathSS, referenceLatlon=referenceLatlon, numLookAheadWaypoints=1))

        # Set state so the boat is not going downwind or upwind
        nextLocalWaypointIndex = 1
        boatPositionXY = latlonToXY(state.position, referenceLatlon)
        desiredHeadingDegrees = math.degrees(math.atan2(solutionPath.getState(nextLocalWaypointIndex).getY() - boatPositionXY[1], solutionPath.getState(nextLocalWaypointIndex).getX() - boatPositionXY[0]))
        validGlobalWindDirectionDegrees = desiredHeadingDegrees + DOWNWIND_MAX_ANGLE_DEGREES * 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=validGlobalWindDirectionDegrees, boatSpeed=1, headingDegrees=120)
        validState = BoatState(globalWaypoint=latlon(1,1), position=latlon(0,0), measuredWindDirectionDegrees=measuredWindDirectionDegrees, measuredWindSpeedKmph=measuredWindSpeedKmph, AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        self.assertFalse(upwindOrDownwindOnPath(state=validState, nextLocalWaypointIndex=nextLocalWaypointIndex, localPathSS=localPathSS, referenceLatlon=referenceLatlon, numLookAheadWaypoints=1))

        # Move boat so that boat is not going downwind or upwind
        nextLocalWaypointIndex = 1
        newPosition = latlon(1, 0)
        boatPositionXY = latlonToXY(newPosition, referenceLatlon)
        desiredHeadingDegrees = math.degrees(math.atan2(solutionPath.getState(nextLocalWaypointIndex).getY() - boatPositionXY[1], solutionPath.getState(nextLocalWaypointIndex).getX() - boatPositionXY[0]))
        validGlobalWindDirectionDegrees = desiredHeadingDegrees + UPWIND_MAX_ANGLE_DEGREES * 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=validGlobalWindDirectionDegrees, boatSpeed=1, headingDegrees=120)
        validState = BoatState(globalWaypoint=latlon(1,1), position=newPosition, measuredWindDirectionDegrees=measuredWindDirectionDegrees, measuredWindSpeedKmph=measuredWindSpeedKmph, AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        self.assertFalse(upwindOrDownwindOnPath(state=validState, nextLocalWaypointIndex=nextLocalWaypointIndex, localPathSS=localPathSS, referenceLatlon=referenceLatlon, numLookAheadWaypoints=1))

        # Move boat so that boat is going downwind
        nextLocalWaypointIndex = 1
        newPosition = latlon(1, 0)
        boatPositionXY = latlonToXY(newPosition, referenceLatlon)
        desiredHeadingDegrees = math.degrees(math.atan2(solutionPath.getState(nextLocalWaypointIndex).getY() - boatPositionXY[1], solutionPath.getState(nextLocalWaypointIndex).getX() - boatPositionXY[0]))
        downwindGlobalWindDirectionDegrees = desiredHeadingDegrees + DOWNWIND_MAX_ANGLE_DEGREES / 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=downwindGlobalWindDirectionDegrees, boatSpeed=1, headingDegrees=120)
        downwindState = BoatState(globalWaypoint=latlon(1,1), position=newPosition, measuredWindDirectionDegrees=measuredWindDirectionDegrees, measuredWindSpeedKmph=measuredWindSpeedKmph, AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        self.assertTrue(upwindOrDownwindOnPath(state=downwindState, nextLocalWaypointIndex=nextLocalWaypointIndex, localPathSS=localPathSS, referenceLatlon=referenceLatlon, numLookAheadWaypoints=1))


    def test_obstacleOnPath(self):
        '''To visualize the obstacleOnPath check, go to updated_geometric_planner.py and uncomment the plotting in hasObstacleOnPath()'''
        # Create simple path from latlon(0,0) to latlon(1,1)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeed=10, globalWindDirectionDegrees=90, boatSpeed=0, headingDegrees=0)
        state = BoatState(globalWaypoint=latlon(1,1), position=latlon(0,0), measuredWindDirectionDegrees=measuredWindDirectionDegrees, measuredWindSpeedKmph=measuredWindSpeedKmph, AISData=AISMsg(), headingDegrees=0, speedKmph=0)
        localPathSS, referenceLatlon = createLocalPathSS(state, runtimeSeconds=3, numRuns=1)

        # No obstacles at all
        self.assertFalse(obstacleOnPath(state=state, nextLocalWaypointIndex=1, localPathSS=localPathSS, referenceLatlon=referenceLatlon))

        # Put obstacle on path between waypoints 1 and 2, going east quickly
        waypoint1 = localPathSS.getSolutionPath().getState(1)
        waypoint2 = localPathSS.getSolutionPath().getState(2)
        shipXY = [waypoint1.getX() + (waypoint2.getX() - waypoint1.getX()) / 2, waypoint1.getY() + (waypoint2.getY() - waypoint1.getY()) / 2]
        between1and2Latlon = XYToLatlon(shipXY, referenceLatlon)
        state.AISData = AISMsg([AISShip(ID=0, lat=between1and2Latlon.lat, lon=between1and2Latlon.lon, headingDegrees=HEADING_EAST, speedKmph=30)])

        # Obstacle on path
        self.assertTrue(obstacleOnPath(state=state, nextLocalWaypointIndex=1, localPathSS=localPathSS, referenceLatlon=referenceLatlon))

        # Move boat position, but still have obstacle on path
        state.position = latlon(-1, -1)
        self.assertTrue(obstacleOnPath(state=state, nextLocalWaypointIndex=1, localPathSS=localPathSS, referenceLatlon=referenceLatlon))

        # Only look ahead 1 waypoint, so no obstacle on that shorter path (defaults to all waypoints if numLookAheadWaypoints not given)
        self.assertFalse(obstacleOnPath(state=state, nextLocalWaypointIndex=1, localPathSS=localPathSS, referenceLatlon=referenceLatlon, numLookAheadWaypoints=1))

        # Move obstacle to be off the path
        between1and2shiftedRightLatlon = XYToLatlon([shipXY[0] + 0.5, shipXY[1]], referenceLatlon)
        state.AISData = AISMsg([AISShip(ID=0, lat=between1and2shiftedRightLatlon.lat, lon=between1and2shiftedRightLatlon.lon, headingDegrees=HEADING_EAST, speedKmph=30)])
        self.assertFalse(obstacleOnPath(state=state, nextLocalWaypointIndex=1, localPathSS=localPathSS, referenceLatlon=referenceLatlon))

        # Move boat position AND increment nextLocalWaypointIndex so it is aiming for waypoint2, so that new "path" has an obstacle on it
        waypoint1Latlon = XYToLatlon([waypoint1.getX(), waypoint1.getY()], referenceLatlon)
        waypoint2Latlon = XYToLatlon([waypoint2.getX(), waypoint2.getY()], referenceLatlon)
        state.position = latlon(waypoint1Latlon.lat, waypoint2Latlon.lon)
        self.assertTrue(obstacleOnPath(state=state, nextLocalWaypointIndex=2, localPathSS=localPathSS, referenceLatlon=referenceLatlon))

        # Move boat position AND increment nextLocalWaypointIndex so it is aiming for waypoint2, so that new "path" doesn't have an obstacle on it
        state.position = latlon(waypoint2Latlon.lat, waypoint1Latlon.lon)
        self.assertFalse(obstacleOnPath(state=state, nextLocalWaypointIndex=2, localPathSS=localPathSS, referenceLatlon=referenceLatlon))

        # Move obstacle to be between waypoint1 and waypoint2, but then have the boat already on waypoint2 going to waypoint3, so no obstacle on it
        state.position = latlon(waypoint2Latlon.lat, waypoint2Latlon.lon)
        state.AISData = AISMsg([AISShip(ID=0, lat=between1and2Latlon.lat, lon=between1and2Latlon.lon, headingDegrees=HEADING_EAST, speedKmph=30)])
        self.assertFalse(obstacleOnPath(state=state, nextLocalWaypointIndex=3, localPathSS=localPathSS, referenceLatlon=referenceLatlon))

if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_utilities', TestUtilities)
