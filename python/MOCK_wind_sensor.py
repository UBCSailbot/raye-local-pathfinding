#!/usr/bin/env python
import rospy
import math

from local_pathfinding.msg import windSensor, GPS
from utilities import globalWindToMeasuredWind
import random

# Constants
WIND_PUBLISH_PERIOD_SECONDS = 10.0
MAX_GLOBAL_WIND_SPEED_KMPH = 10
MIN_GLOBAL_WIND_SPEED_KMPH = 1
CHANGE_WIND_DIRECTION_PERIOD_SECONDS = 3600.0 * 3
CHANGE_WIND_DIRECTION_AMOUNT_DEGREES = 45
CHANGE_COUNTER_MAX = 2

# Global variables for tracking boat speed
boatHeadingDegrees = 0
boatSpeedKmph = 0
changeCounter = 0
first = True
maybeNewHeadingDegrees = 0
def gpsCallback(data):
    global boatHeadingDegrees
    global boatSpeedKmph
    global changeCounter
    global first
    global raybeNewHeadingDegrees

    if math.fabs(maybeNewHeadingDegrees - data.headingDegrees) < 1:
        changeCounter += 1
    else:
        maybeNewHeadingDegrees = data.headingDegrees
        changeCounter = 0
    rospy.logerr("=================changeCounter = {}".format(changeCounter))
    rospy.logerr("=================boatHeadingDegrees = {}".format(boatHeadingDegrees))
    rospy.logerr("=================maybeNewHeadingDegrees = {}".format(maybeNewHeadingDegrees))

    # Update global variables only if:
    # 1. the new values hold for a certain amount of time OR
    # 2. this is the first callback
    if changeCounter >= CHANGE_COUNTER_MAX or first:
        rospy.logerr("=================UPDATING boatHeadingDegrees in wind_sensor")
        boatHeadingDegrees = maybeNewHeadingDegrees
        boatSpeedKmph = data.speedKmph
        first = False
        changeCounter = 0

def talker():
    global boatHeadingDegrees
    global boatSpeedKmph

    pub = rospy.Publisher('windSensor', windSensor, queue_size=4)
    rospy.init_node('MOCK_wind_talker', anonymous=True)
    r = rospy.Rate(1.0 / WIND_PUBLISH_PERIOD_SECONDS)
    msg = windSensor()

    # Get speedup parameter
    speedup = rospy.get_param('speedup', default=1.0)

    # Set initial global wind conditions
    globalWindSpeedKmph = random.randint(MIN_GLOBAL_WIND_SPEED_KMPH, MAX_GLOBAL_WIND_SPEED_KMPH)
    globalWindDirectionDegrees = 90

    # Change wind periodically and
    # Change wind more often with speedup
    changeWindCounter = 0
    changeWindDirectionPeriodSecondsSpeedup = CHANGE_WIND_DIRECTION_PERIOD_SECONDS / speedup
    changeWindMax = int(changeWindDirectionPeriodSecondsSpeedup / WIND_PUBLISH_PERIOD_SECONDS)
    while not rospy.is_shutdown():
        # Update global wind speed and direction
        if changeWindCounter >= changeWindMax:
            rospy.logerr("=================UPDATING WIND DIRECTION in wind_sensor")
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
        rospy.logerr(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber("GPS", GPS, gpsCallback)
        talker()
    except rospy.ROSInterruptException: pass
