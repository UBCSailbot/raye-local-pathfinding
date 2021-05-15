#! /usr/bin/env python

import local_imports  # Must be first import, as it adds python directory to path
import Sailbot as sbot
import sailbot_msg.msg as msg
import rospy
import rostest
import unittest

# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class TestSailbot(unittest.TestCase):
    def setUp(self):
        # Setup sailbot object
        self.sailbot = sbot.Sailbot(nodeName='test_sailbot')

        # Setup publishers
        self.gpsPublisher = rospy.Publisher("GPS", msg.GPS, queue_size=4)
        self.globalWindSensorPublisher = rospy.Publisher("global_wind", msg.globalWind, queue_size=4)
        self.AISPublisher = rospy.Publisher("AIS", msg.AISMsg, queue_size=4)
        self.globalPathPublisher = rospy.Publisher("globalPath", msg.path, queue_size=4)

        # Wait for connections to be made
        while self.gpsPublisher.get_num_connections() == 0:
            rospy.sleep(0.001)
        while self.globalWindSensorPublisher.get_num_connections() == 0:
            rospy.sleep(0.001)
        while self.AISPublisher.get_num_connections() == 0:
            rospy.sleep(0.001)
        while self.globalPathPublisher.get_num_connections() == 0:
            rospy.sleep(0.001)

    def test_basic(self):
        # Setup messages
        gpsMsg = msg.GPS(10.1, 4.2, 60.3, 12.4)
        globalWindMsg = msg.globalWind(87.5, 1.89)
        ships = [msg.AISShip(i, i, i, i, i) for i in range(10)]
        AISMsg = msg.AISMsg(ships)
        waypoints = [msg.latlon(i, i) for i in range(10)]
        globalPathMsg = msg.path(waypoints)

        # Publish messages
        self.gpsPublisher.publish(gpsMsg)
        self.globalWindSensorPublisher.publish(globalWindMsg)
        self.AISPublisher.publish(AISMsg)
        self.globalPathPublisher.publish(globalPathMsg)
        rospy.sleep(1)

        # Check that currentState has been updated
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.position.lat, gpsMsg.lat, places=3)
        self.assertAlmostEqual(state.position.lon, gpsMsg.lon, places=3)
        self.assertAlmostEqual(state.headingDegrees, gpsMsg.headingDegrees, places=3)
        self.assertAlmostEqual(state.speedKmph, gpsMsg.speedKmph, places=3)
        self.assertAlmostEqual(state.globalWindDirectionDegrees, globalWindMsg.directionDegrees, places=3)
        self.assertAlmostEqual(state.globalWindSpeedKmph, globalWindMsg.speedKmph, places=3)
        self.assertEqual(len(state.AISData.ships), len(ships))
        self.assertEqual(len(self.sailbot.globalPath), len(waypoints))

    def test_missing_data(self):
        # Setup messages
        gpsMsg = msg.GPS(10.1, 4.2, 60.3, 12.4)
        globalWindMsg = msg.globalWind(87.5, 1.89)
        ships = [msg.AISShip(i, i, i, i, i) for i in range(10)]
        AISMsg = msg.AISMsg(ships)
        waypoints = [msg.latlon(i, i) for i in range(10)]
        globalPathMsg = msg.path(waypoints)

        # Ensure that state gives None unless all data has been received
        self.assertIsNone(self.sailbot.getCurrentState())

        self.gpsPublisher.publish(gpsMsg)
        rospy.sleep(0.1)
        self.assertIsNone(self.sailbot.getCurrentState())

        self.globalWindSensorPublisher.publish(globalWindMsg)
        rospy.sleep(0.1)
        self.assertIsNone(self.sailbot.getCurrentState())

        self.AISPublisher.publish(AISMsg)
        rospy.sleep(0.1)
        self.assertIsNone(self.sailbot.getCurrentState())

        # Check that currentState has been updated after all data has been received
        self.globalPathPublisher.publish(globalPathMsg)
        rospy.sleep(0.1)

        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.position.lat, gpsMsg.lat, places=3)
        self.assertAlmostEqual(state.position.lon, gpsMsg.lon, places=3)
        self.assertAlmostEqual(state.headingDegrees, gpsMsg.headingDegrees, places=3)
        self.assertAlmostEqual(state.speedKmph, gpsMsg.speedKmph, places=3)
        self.assertAlmostEqual(state.globalWindDirectionDegrees, globalWindMsg.directionDegrees, places=3)
        self.assertAlmostEqual(state.globalWindSpeedKmph, globalWindMsg.speedKmph, places=3)
        self.assertEqual(len(state.AISData.ships), len(ships))
        self.assertEqual(len(self.sailbot.globalPath), len(waypoints))

    def test_globalPath_detailed(self):
        # Setup non global path messages
        gpsMsg = msg.GPS(10.1, 4.2, 60.3, 12.4)
        globalWindMsg = msg.globalWind(87.5, 1.89)
        ships = [msg.AISShip(i, i, i, i, i) for i in range(10)]
        AISMsg = msg.AISMsg(ships)

        # Publish non global path messages
        self.gpsPublisher.publish(gpsMsg)
        self.globalWindSensorPublisher.publish(globalWindMsg)
        self.AISPublisher.publish(AISMsg)
        rospy.sleep(1)

        # Setup globalPath message
        numWaypoints = 10
        waypoints = [msg.latlon(i, i) for i in range(numWaypoints)]
        globalPathMsg = msg.path(waypoints)

        self.globalPathPublisher.publish(globalPathMsg)
        rospy.sleep(0.1)

        # Check that sailbot received the new path
        self.assertEqual(len(self.sailbot.globalPath), numWaypoints)

        # Check that currentState has been updated next global waypoint should be
        # the second element of the waypoints list
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.globalWaypoint.lat, waypoints[1].lat, places=3)
        self.assertAlmostEqual(state.globalWaypoint.lon, waypoints[1].lon, places=3)

        # Check that globalIndex increment works
        self.sailbot.globalPathIndex += 1
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.globalWaypoint.lat, waypoints[2].lat, places=3)
        self.assertAlmostEqual(state.globalWaypoint.lon, waypoints[2].lon, places=3)

        # Check that receiving the same path doesn't update the index to 1
        self.globalPathPublisher.publish(globalPathMsg)
        rospy.sleep(0.1)
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.globalWaypoint.lat, waypoints[2].lat, places=3)
        self.assertAlmostEqual(state.globalWaypoint.lon, waypoints[2].lon, places=3)

        # Check that receiving a new path does update the index to 1
        waypoints = [msg.latlon(i + 1, i + 1) for i in range(numWaypoints)]
        globalPathMsg = msg.path(waypoints)
        self.globalPathPublisher.publish(globalPathMsg)
        rospy.sleep(0.1)
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.globalWaypoint.lat, waypoints[1].lat, places=3)
        self.assertAlmostEqual(state.globalWaypoint.lon, waypoints[1].lon, places=3)

        # Check that globalIndex increment out of range returns last waypoint
        self.sailbot.globalPathIndex = numWaypoints + 1
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.globalWaypoint.lat, waypoints[numWaypoints - 1].lat, places=3)
        self.assertAlmostEqual(state.globalWaypoint.lon, waypoints[numWaypoints - 1].lon, places=3)


if __name__ == '__main__':
    rostest.rosrun('local_pathfinding', 'test_sailbot', TestSailbot)
