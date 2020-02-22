#! /usr/bin/env python

# Add python directory to path
import sys, os
testdir = os.path.dirname(__file__)
srcdir = '../python'
sys.path.insert(0, os.path.abspath(os.path.join(testdir, srcdir)))

import unittest
import rostest
import rospy
import local_pathfinding.msg as msg
from utilities import *
from Sailbot import *
from geopy.distance import distance
from math import sqrt
import matplotlib.pyplot as plt

class TestSailbot(unittest.TestCase):
    def setUp(self):
        # Setup sailbot object
        self.sailbot = Sailbot(nodeName='test_sailbot')

    def test_GPS(self):
        # Setup GPS message
        gpsMsg = msg.GPS()
        gpsMsg.lat = 10.1
        gpsMsg.lon = 4.2
        gpsMsg.headingDegrees = 60.3
        gpsMsg.speedKmph = 12.4

        # Publish GPS message
        gpsPublisher = rospy.Publisher("GPS", msg.GPS, queue_size=4)
        # Wait for connection to be made between subscriber and publisher
        while gpsPublisher.get_num_connections() == 0:
            rospy.sleep(0.001)
        gpsPublisher.publish(gpsMsg)
        rospy.sleep(0.1)

        # Check that currentState has been updated
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.position.lat, gpsMsg.lat, places=3)
        self.assertAlmostEqual(state.position.lon, gpsMsg.lon, places=3)
        self.assertAlmostEqual(state.headingDegrees, gpsMsg.headingDegrees, places=3)
        self.assertAlmostEqual(state.speedKmph, gpsMsg.speedKmph, places=3)

    def test_windSensor(self):
        # Setup windSensor message
        windSensorMsg = msg.windSensor()
        windSensorMsg.measuredDirectionDegrees = 87.5
        windSensorMsg.measuredSpeedKmph = 1.89

        # Publish windsensor message
        windSensorPublisher = rospy.Publisher("windSensor", msg.windSensor, queue_size=4)
        # Wait for connection to be made between subscriber and publisher
        while windSensorPublisher.get_num_connections() == 0:
            rospy.sleep(0.001)
        windSensorPublisher.publish(windSensorMsg)
        rospy.sleep(0.1)

        # Check that currentState has been updated
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.measuredWindDirectionDegrees, windSensorMsg.measuredDirectionDegrees, places=3)
        self.assertAlmostEqual(state.measuredWindSpeedKmph, windSensorMsg.measuredSpeedKmph, places=3)

    def test_AIS(self):
        # Setup AIS message with multiple ships
        numShips = 10
        ships = [msg.AISShip(i, i, i, i, i) for i in range(numShips)]
        AISMsg = msg.AISMsg(ships)

        # Publish AIS message
        AISPublisher = rospy.Publisher("AIS", msg.AISMsg, queue_size=4)
        # Wait for connection to be made between subscriber and publisher
        while AISPublisher.get_num_connections() == 0:
            rospy.sleep(0.001)
        AISPublisher.publish(AISMsg)
        rospy.sleep(0.1)

        # Check that currentState has been updated
        state = self.sailbot.getCurrentState()
        self.assertEqual(len(state.AISData.ships), numShips)

    def test_globalPath(self):
        # Setup globalPath message
        numWaypoints = 10
        waypoints = [latlon(i, i) for i in range(numWaypoints)]
        globalPathMsg = msg.path(waypoints)

        # Publish globalPath message
        globalPathPublisher = rospy.Publisher("globalPath", msg.path, queue_size=4)
        # Wait for connection to be made between subscriber and publisher
        while globalPathPublisher.get_num_connections() == 0:
            rospy.sleep(0.001)
        globalPathPublisher.publish(globalPathMsg)
        rospy.sleep(0.1)

        # Check that sailbot received the new path
        self.assertEqual(len(self.sailbot.globalPath), numWaypoints)

        # Check that currentState has been updated next global waypoint should be the second element of the waypoints list
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.globalWaypoint.lat, waypoints[1].lat, places=3)
        self.assertAlmostEqual(state.globalWaypoint.lon, waypoints[1].lon, places=3)

        # Check that globalIndex increment works
        self.sailbot.globalPathIndex += 1
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.globalWaypoint.lat, waypoints[2].lat, places=3)
        self.assertAlmostEqual(state.globalWaypoint.lon, waypoints[2].lon, places=3)

        # Check that receiving the same path doesn't update the index to 1
        globalPathPublisher.publish(globalPathMsg)
        rospy.sleep(0.1)
        state = self.sailbot.getCurrentState()
        self.assertAlmostEqual(state.globalWaypoint.lat, waypoints[2].lat, places=3)
        self.assertAlmostEqual(state.globalWaypoint.lon, waypoints[2].lon, places=3)

        # Check that receiving a new path does update the index to 1
        waypoints = [latlon(i + 1, i + 1) for i in range(numWaypoints)]
        globalPathMsg = msg.path(waypoints)
        globalPathPublisher.publish(globalPathMsg)
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
