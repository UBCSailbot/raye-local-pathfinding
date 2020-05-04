#! /usr/bin/env python

import local_imports  # Must be first import, as it adds python directory to path
import utilities as utils
import Sailbot as sbot
from local_pathfinding.msg import latlon, AISMsg, AISShip
from planner_helpers import UPWIND_MAX_ANGLE_DEGREES, DOWNWIND_MAX_ANGLE_DEGREES
from Path import UPWIND_DOWNWIND_TIME_LIMIT_SECONDS
import rostest
import unittest
import time
from ompl import geometric as og

# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class TestPath(unittest.TestCase):
    def createPathWithGivenPositionsXY(self, xys):
        def createState(spaceInformation, xy):
            state = spaceInformation.allocState()
            state.setXY(xy[0], xy[1])
            return state

        # Create dummy path and change private variables directly
        state = sbot.BoatState(globalWaypoint=latlon(0.2, 0.2), position=latlon(0, 0),
                               measuredWindDirectionDegrees=90, measuredWindSpeedKmph=10,
                               AISData=AISMsg(), headingDegrees=0, speedKmph=0)
        path = utils.createPath(state, runtimeSeconds=1, numRuns=2)
        spaceInformation = path.getSpaceInformation()

        # Create acutal path
        actualPath = og.PathGeometric(spaceInformation)
        for xy in xys:
            actualPath.append(createState(spaceInformation, xy))
        path.getOMPLPath()._solutionPath = actualPath
        path._latlons = path._getLatlonsFromOMPLPath(path._omplPath)

        return path

    def test_localWaypointReached(self):
        # Setup path from (0,0) to (1,1)
        path = self.createPathWithGivenPositionsXY([[0, 0], [1, 1]])

        # Test reachedPos
        reachedPos = utils.XYToLatlon(xy=(2, 2), referenceLatlon=path.getReferenceLatlon())
        self.assertTrue(path.nextWaypointReached(reachedPos))

        # Test notReachedPos
        notReachedPos = utils.XYToLatlon(xy=(1, 0.5), referenceLatlon=path.getReferenceLatlon())
        self.assertFalse(path.nextWaypointReached(notReachedPos))

    # testing cases where start and LWP have same lat or same lon
    def test_localWaypointReached_sameLat(self):
        # Setup path from (1,1) to (1,2)
        path = self.createPathWithGivenPositionsXY([[1, 1], [1, 2]])

        # Test reachedPos
        reachedPos = utils.XYToLatlon(xy=(100, 2.1), referenceLatlon=path.getReferenceLatlon())
        self.assertTrue(path.nextWaypointReached(reachedPos))

        # Test notReachedPos
        notReachedPos = utils.XYToLatlon(xy=(-100, 1.9), referenceLatlon=path.getReferenceLatlon())
        self.assertFalse(path.nextWaypointReached(notReachedPos))

    def test_localWaypointReached_sameLon(self):
        # Setup path from (2,1) to (1,1)
        path = self.createPathWithGivenPositionsXY([[2, 1], [1, 1]])

        # Test reachedPos
        reachedPos = utils.XYToLatlon(xy=(0.9, -100), referenceLatlon=path.getReferenceLatlon())
        self.assertTrue(path.nextWaypointReached(reachedPos))

        # Test notReachedPos
        notReachedPos = utils.XYToLatlon(xy=(1.1, 100), referenceLatlon=path.getReferenceLatlon())
        self.assertFalse(path.nextWaypointReached(notReachedPos))

    def test_upwindOrDownwindOnPath(self):
        # Create simple path from latlon(0,0) to latlon(0.2,0.2)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=90, boatSpeed=0, headingDegrees=0)
        state = sbot.BoatState(globalWaypoint=latlon(0.2, 0.2), position=latlon(0, 0),
                               measuredWindDirectionDegrees=measuredWindDirectionDegrees,
                               measuredWindSpeedKmph=measuredWindSpeedKmph,
                               AISData=AISMsg(), headingDegrees=0, speedKmph=0)
        path = utils.createPath(state, runtimeSeconds=1, numRuns=2)
        desiredHeadingDegrees = utils.getDesiredHeadingDegrees(state.position, path.getNextWaypoint())

        # Set state with global wind direction nearly same as boat current direction (sailing downwind)
        downwindGlobalWindDirectionDegrees = desiredHeadingDegrees + DOWNWIND_MAX_ANGLE_DEGREES / 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=downwindGlobalWindDirectionDegrees, boatSpeed=1,
            headingDegrees=120)
        downwindState = sbot.BoatState(globalWaypoint=latlon(0.2, 0.2), position=latlon(0, 0),
                                       measuredWindDirectionDegrees=measuredWindDirectionDegrees,
                                       measuredWindSpeedKmph=measuredWindSpeedKmph,
                                       AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        # Needs to maintain downwind for time limit before returning true
        self.assertFalse(path.upwindOrDownwindOnPath(downwindState, numLookAheadWaypoints=1))
        time.sleep(UPWIND_DOWNWIND_TIME_LIMIT_SECONDS * 1.5)
        self.assertTrue(path.upwindOrDownwindOnPath(downwindState, numLookAheadWaypoints=1))

        # Set state with global wind direction nearly 180 degrees from boat current direction (sailing upwind)
        upwindGlobalWindDirectionDegrees = desiredHeadingDegrees + 180 + UPWIND_MAX_ANGLE_DEGREES / 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=upwindGlobalWindDirectionDegrees, boatSpeed=1,
            headingDegrees=120)
        upwindState = sbot.BoatState(globalWaypoint=latlon(0.2, 0.2), position=latlon(0, 0),
                                     measuredWindDirectionDegrees=measuredWindDirectionDegrees,
                                     measuredWindSpeedKmph=measuredWindSpeedKmph,
                                     AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        self.assertFalse(path.upwindOrDownwindOnPath(upwindState, numLookAheadWaypoints=1))
        time.sleep(UPWIND_DOWNWIND_TIME_LIMIT_SECONDS * 1.5)
        self.assertTrue(path.upwindOrDownwindOnPath(upwindState, numLookAheadWaypoints=1))

        # Set state so the boat is not going downwind or upwind
        validGlobalWindDirectionDegrees = desiredHeadingDegrees + DOWNWIND_MAX_ANGLE_DEGREES * 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=validGlobalWindDirectionDegrees, boatSpeed=1,
            headingDegrees=120)
        validState = sbot.BoatState(globalWaypoint=latlon(0.2, 0.2), position=latlon(0, 0),
                                    measuredWindDirectionDegrees=measuredWindDirectionDegrees,
                                    measuredWindSpeedKmph=measuredWindSpeedKmph,
                                    AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        self.assertFalse(path.upwindOrDownwindOnPath(validState, numLookAheadWaypoints=1))
        time.sleep(UPWIND_DOWNWIND_TIME_LIMIT_SECONDS * 1.5)
        self.assertFalse(path.upwindOrDownwindOnPath(validState, numLookAheadWaypoints=1))

        # Move boat so that boat is not going downwind or upwind
        newPosition = latlon(0.2, 0)
        desiredHeadingDegrees = utils.getDesiredHeadingDegrees(newPosition, path.getNextWaypoint())
        validGlobalWindDirectionDegrees = desiredHeadingDegrees + UPWIND_MAX_ANGLE_DEGREES * 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=validGlobalWindDirectionDegrees, boatSpeed=1,
            headingDegrees=120)
        validState = sbot.BoatState(globalWaypoint=latlon(0.2, 0.2), position=newPosition,
                                    measuredWindDirectionDegrees=measuredWindDirectionDegrees,
                                    measuredWindSpeedKmph=measuredWindSpeedKmph,
                                    AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        self.assertFalse(path.upwindOrDownwindOnPath(validState, numLookAheadWaypoints=1))
        time.sleep(UPWIND_DOWNWIND_TIME_LIMIT_SECONDS * 1.5)
        self.assertFalse(path.upwindOrDownwindOnPath(validState, numLookAheadWaypoints=1))

        # Move boat so that boat is going downwind
        newPosition = latlon(0, 0.2)
        desiredHeadingDegrees = utils.getDesiredHeadingDegrees(newPosition, path.getNextWaypoint())
        downwindGlobalWindDirectionDegrees = desiredHeadingDegrees + DOWNWIND_MAX_ANGLE_DEGREES / 2

        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=downwindGlobalWindDirectionDegrees, boatSpeed=1,
            headingDegrees=120)
        invalidState = sbot.BoatState(globalWaypoint=latlon(0.2, 0.2), position=newPosition,
                                      measuredWindDirectionDegrees=measuredWindDirectionDegrees,
                                      measuredWindSpeedKmph=measuredWindSpeedKmph,
                                      AISData=AISMsg(), headingDegrees=120, speedKmph=1)
        self.assertFalse(path.upwindOrDownwindOnPath(invalidState, numLookAheadWaypoints=1))
        time.sleep(UPWIND_DOWNWIND_TIME_LIMIT_SECONDS * 1.5)
        self.assertTrue(path.upwindOrDownwindOnPath(invalidState, numLookAheadWaypoints=1))

    def test_obstacleOnPath(self):
        '''To visualize the obstacleOnPath check, go to updated_geometric_planner.py
           and uncomment the plotting in hasObstacleOnPath()'''
        # Create simple path from latlon(0,0) to latlon(0.2,0.2)
        measuredWindSpeedKmph, measuredWindDirectionDegrees = utils.globalWindToMeasuredWind(
            globalWindSpeed=10, globalWindDirectionDegrees=90, boatSpeed=0, headingDegrees=0)
        state = sbot.BoatState(globalWaypoint=latlon(0.2, 0.2), position=latlon(0, 0),
                               measuredWindDirectionDegrees=measuredWindDirectionDegrees,
                               measuredWindSpeedKmph=measuredWindSpeedKmph,
                               AISData=AISMsg(), headingDegrees=0, speedKmph=0)
        path = utils.createPath(state, runtimeSeconds=1, numRuns=2)

        waypoint0 = path.getOMPLPath().getSolutionPath().getState(0)
        waypoint1 = path.getOMPLPath().getSolutionPath().getState(1)
        waypoint2 = path.getOMPLPath().getSolutionPath().getState(2)

        # No obstacles at all
        self.assertFalse(path.obstacleOnPath(state))

        # Put obstacle on path between waypoints 0 and 1, going east quickly
        between0and1XY = [waypoint0.getX() + (waypoint1.getX() - waypoint0.getX()) / 2,
                          waypoint0.getY() + (waypoint1.getY() - waypoint0.getY()) / 2]
        between0and1Latlon = utils.XYToLatlon(between0and1XY, path.getReferenceLatlon())
        state.AISData = AISMsg([AISShip(ID=0, lat=between0and1Latlon.lat, lon=between0and1Latlon.lon,
                                        headingDegrees=utils.HEADING_EAST, speedKmph=30)])

        # Obstacle on path
        self.assertTrue(path.obstacleOnPath(state))

        # Only look ahead 1 waypoint, but still has obstacle on path
        # (defaults to all waypoints if numLookAheadWaypoints not given)
        self.assertTrue(path.obstacleOnPath(state, numLookAheadWaypoints=1))

        # Move obstacle to be off the path
        between0and1shiftedRightXY = [waypoint0.getX() + (waypoint1.getX() - waypoint0.getX()) * 3 / 4,
                                      waypoint0.getY() + (waypoint1.getY() - waypoint0.getY()) / 2]
        between0and1shiftedRightLatlon = utils.XYToLatlon(between0and1shiftedRightXY, path.getReferenceLatlon())
        state.AISData = AISMsg([AISShip(ID=0, lat=between0and1shiftedRightLatlon.lat,
                                        lon=between0and1shiftedRightLatlon.lon,
                                        headingDegrees=utils.HEADING_EAST, speedKmph=30)])
        self.assertFalse(path.obstacleOnPath(state))

        # Move boat position, so that new "path" has an obstacle on it
        waypoint0Latlon = utils.XYToLatlon([waypoint0.getX(), waypoint0.getY()], path.getReferenceLatlon())
        waypoint1Latlon = utils.XYToLatlon([waypoint1.getX(), waypoint1.getY()], path.getReferenceLatlon())
        state.position = latlon(waypoint0Latlon.lat, waypoint1Latlon.lon)
        self.assertTrue(path.obstacleOnPath(state))

        # Move boat position, so that new "path" does not have an obstacle on it
        state.position = latlon(waypoint1Latlon.lat, waypoint0Latlon.lon)
        self.assertFalse(path.obstacleOnPath(state))

        between1and2XY = [waypoint1.getX() + (waypoint2.getX() - waypoint1.getX()) / 2,
                          waypoint1.getY() + (waypoint2.getY() - waypoint1.getY()) / 2]
        between1and2Latlon = utils.XYToLatlon(between1and2XY, path.getReferenceLatlon())
        state.AISData = AISMsg([AISShip(ID=0, lat=between1and2Latlon.lat, lon=between1and2Latlon.lon,
                                        headingDegrees=utils.HEADING_EAST, speedKmph=30)])

        # Obstacle on path
        self.assertTrue(path.obstacleOnPath(state))

        # Only look ahead 1 waypoint, so does not have obstacle on path
        # (defaults to all waypoints if numLookAheadWaypoints not given)
        self.assertFalse(path.obstacleOnPath(state, numLookAheadWaypoints=1))


if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_Path', TestPath)
