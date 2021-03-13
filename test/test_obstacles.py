#!/usr/bin/env python
import local_imports  # Must be first import, as it adds python directory to path
from sailbot_msg.msg import latlon, AISMsg, AISShip
import utilities as utils
import obstacles as obs
import Sailbot as sbot
import rostest
import unittest

# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class TestObstacles(unittest.TestCase):
    def plotObstacles(self, obstacles):
        import matplotlib.pyplot as plt
        ax = plt.gca()
        for obstacle in obstacles:
            obstacle.addPatch(ax)
        plt.show()

    def test_getObstacles(self):
        # Setup: sailbot starting at (0,0). obstacle at (1,0) heading west
        currentLatlon = latlon(0, 0)
        referenceLatlon = currentLatlon
        shipLatlon = utils.XYToLatlon(xy=(1, 0), referenceLatlon=referenceLatlon)
        ships = [AISShip(ID=1000, lat=shipLatlon.lat, lon=shipLatlon.lon, headingDegrees=utils.HEADING_WEST,
                         speedKmph=1)]
        state = sbot.BoatState(globalWaypoint=latlon(1, 1), position=currentLatlon, measuredWindDirectionDegrees=0,
                               measuredWindSpeedKmph=0, AISData=AISMsg(ships), headingDegrees=0, speedKmph=1)
        obstacles = obs.getObstacles(state, referenceLatlon=referenceLatlon)

        # Uncomment below to see obstacles extended on a plot
        # self.plotObstacles(obstacles)

        self.assertFalse(utils.isValid(xy=[0.5, 0], obstacles=obstacles))
        self.assertTrue(utils.isValid(xy=[0, 0.5], obstacles=obstacles))
        self.assertTrue(utils.isValid(xy=[0, -0.5], obstacles=obstacles))

    def test_getObstacles2(self):
        # Setup starting at (0,0) with obstacle at (1,1) heading south-west
        currentLatlon = latlon(0, 0)
        referenceLatlon = currentLatlon
        shipLatlon = utils.XYToLatlon(xy=(1, 1), referenceLatlon=referenceLatlon)
        ships = [AISShip(ID=1000, lat=shipLatlon.lat, lon=shipLatlon.lon,
                         headingDegrees=utils.HEADING_SOUTH, speedKmph=1)]
        state = sbot.BoatState(globalWaypoint=latlon(1, 1), position=currentLatlon, measuredWindDirectionDegrees=0,
                               measuredWindSpeedKmph=0, AISData=AISMsg(ships), headingDegrees=0, speedKmph=1)
        obstacles = obs.getObstacles(state, referenceLatlon=referenceLatlon)

        # Uncomment below to see obstacles extended on a plot
        # self.plotObstacles(obstacles)

        self.assertFalse(utils.isValid(xy=[1, 1], obstacles=obstacles))
        self.assertTrue(utils.isValid(xy=[0, 0], obstacles=obstacles))
        self.assertTrue(utils.isValid(xy=[1, 0], obstacles=obstacles))
        self.assertTrue(utils.isValid(xy=[0, 1], obstacles=obstacles))

    def test_getObstacles3(self):
        # Setup starting at (0,0) with obstacles at (0,3) heading south and at (-1,-1) heading north east
        currentLatlon = latlon(0, 0)
        referenceLatlon = currentLatlon
        shipLatlon = utils.XYToLatlon(xy=(0, 3), referenceLatlon=referenceLatlon)
        shipLatlon2 = utils.XYToLatlon(xy=(-1, -1), referenceLatlon=referenceLatlon)
        ship1 = AISShip(ID=1000, lat=shipLatlon.lat, lon=shipLatlon.lon, headingDegrees=270, speedKmph=1.5)
        ship2 = AISShip(ID=1001, lat=shipLatlon2.lat, lon=shipLatlon2.lon, headingDegrees=45, speedKmph=10)
        state = sbot.BoatState(globalWaypoint=latlon(1, 1), position=currentLatlon, measuredWindDirectionDegrees=0,
                               measuredWindSpeedKmph=0, AISData=AISMsg([ship1, ship2]), headingDegrees=0, speedKmph=1)
        obstacles = obs.getObstacles(state, referenceLatlon=referenceLatlon)

        # Uncomment below to see obstacles extended on a plot
        # self.plotObstacles(obstacles)

        self.assertFalse(utils.isValid(xy=[0, 0], obstacles=obstacles))
        self.assertFalse(utils.isValid(xy=[1, 1], obstacles=obstacles))
        self.assertFalse(utils.isValid(xy=[2.5, 2.5], obstacles=obstacles))
        self.assertTrue(utils.isValid(xy=[3, 3], obstacles=obstacles))
        self.assertTrue(utils.isValid(xy=[0, -1], obstacles=obstacles))
        self.assertTrue(utils.isValid(xy=[0, 4], obstacles=obstacles))
        self.assertTrue(utils.isValid(xy=[0, -2], obstacles=obstacles))


if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_obstacles', TestObstacles)
