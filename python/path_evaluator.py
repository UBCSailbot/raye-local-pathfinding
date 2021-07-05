#!/usr/bin/env python
import Path
import Sailbot as sbot
import rospy

sailbot = sbot.Sailbot(nodeName='path_evaluator')
sailbot.waitForFirstSensorDataAndGlobalPath()
state = sailbot.getCurrentState()
path = Path.createPath(state)
rospy.logwarn("Path evaluator got cost = {}".format(path.getCost()))
