#!/usr/bin/env python
import rospy
import math
import json

from std_msgs.msg import Float64
from local_pathfinding.msg import windSensor, GPS, globalWind
from utilities import globalWindToMeasuredWind
import random

# Constants
WIND_PUBLISH_PERIOD_SECONDS = 0.1  # Must keep well below 1.0 to ensure proper global wind direction calculation, even when the boat is turning
MAX_GLOBAL_WIND_SPEED_KMPH = 10
MIN_GLOBAL_WIND_SPEED_KMPH = 1
CHANGE_WIND_DIRECTION_PERIOD_SECONDS = 3600.0 * 3
CHANGE_WIND_DIRECTION_AMOUNT_DEGREES = 45

# Global variables for tracking boat speed
boatHeadingDegrees = 0
boatSpeedKmph = 0
def gpsCallback(data):
    global boatHeadingDegrees
    global boatSpeedKmph
    boatHeadingDegrees = data.headingDegrees
    boatSpeedKmph = data.speedKmph

# Global variables for global wind conditions
globalWindSpeedKmph = random.randint(MIN_GLOBAL_WIND_SPEED_KMPH, MAX_GLOBAL_WIND_SPEED_KMPH)
globalWindDirectionDegrees = 90
def globalWindCallback(data):
    global globalWindSpeedKmph
    global globalWindDirectionDegrees
    rospy.loginfo("Received change global wind message = {}".format(data))
    globalWindSpeedKmph = data.speedKmph
    globalWindDirectionDegrees = data.directionDegrees

# Global variable for speedup
speedup = 1.0
def speedupCallback(data):
    global speedup
    speedup = data.data

def talker():
    global boatHeadingDegrees
    global boatSpeedKmph
    global globalWindSpeedKmph
    global globalWindDirectionDegrees

    pub = rospy.Publisher('windSensor', windSensor, queue_size=4)
    rospy.init_node('MOCK_wind_talker', anonymous=True)
    r = rospy.Rate(1.0 / WIND_PUBLISH_PERIOD_SECONDS)
    msg = windSensor()

    # Get wind_file parameter
    wind_file = rospy.get_param('wind_file', default=None)
    if wind_file:
        with open(wind_file) as f:
            record = json.loads(f.read())
            globalWindSpeedKmph = record[0]
            globalWindDirectionDegrees = record[1]

    # Change wind periodically and
    # Change wind more often with speedup
    changeWindCounter = 0
    changeWindDirectionPeriodSecondsSpeedup = CHANGE_WIND_DIRECTION_PERIOD_SECONDS / speedup
    changeWindMax = int(changeWindDirectionPeriodSecondsSpeedup / WIND_PUBLISH_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        # Update global wind speed and direction
        if changeWindCounter >= changeWindMax:
            changeWindCounter = 0
            globalWindSpeedKmph = random.randint(MIN_GLOBAL_WIND_SPEED_KMPH, MAX_GLOBAL_WIND_SPEED_KMPH)
            globalWindDirectionDegrees += CHANGE_WIND_DIRECTION_AMOUNT_DEGREES
        else:
            changeWindCounter += 1

        # Set windSensor message such that the global wind matches the variables
        measuredWindSpeedKmph, measuredWindDirectionDegrees = globalWindToMeasuredWind(globalWindSpeedKmph, globalWindDirectionDegrees, boatSpeedKmph, boatHeadingDegrees)
        msg.measuredDirectionDegrees = measuredWindDirectionDegrees
        msg.measuredSpeedKmph = measuredWindSpeedKmph
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber("GPS", GPS, gpsCallback)
        rospy.Subscriber("speedup", Float64, speedupCallback)
        rospy.Subscriber("changeGlobalWind", globalWind, globalWindCallback)
        talker()
    except rospy.ROSInterruptException: pass
