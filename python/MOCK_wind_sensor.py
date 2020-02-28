#!/usr/bin/env python
import rospy
import math

from local_pathfinding.msg import windSensor, GPS
from utilities import globalWindToMeasuredWind
import random

# Constants
WIND_PUBLISH_PERIOD_SECONDS = 15.0
MAX_GLOBAL_WIND_SPEED_KMPH = 10
MIN_GLOBAL_WIND_SPEED_KMPH = 1
CHANGE_WIND_DIRECTION_PERIOD_SECONDS = 60
CHANGE_WIND_DIRECTION_AMOUNT_DEGREES = 45

# Global variables for tracking boat speed
boatHeadingDegrees = 0
boatSpeedKmph = 0
def gpsCallback(data):
    global boatHeadingDegrees
    global boatSpeedKmph
    boatHeadingDegrees = data.headingDegrees
    boatSpeedKmph = data.speedKmph

def talker():
    global boatHeadingDegrees
    global boatSpeedKmph

    pub = rospy.Publisher('windSensor', windSensor, queue_size=4)
    rospy.init_node('MOCK_wind_talker', anonymous=True)
    r = rospy.Rate(1.0 / WIND_PUBLISH_PERIOD_SECONDS)
    msg = windSensor()

    # Set initial global wind conditions
    globalWindSpeedKmph = random.randint(MIN_GLOBAL_WIND_SPEED_KMPH, MAX_GLOBAL_WIND_SPEED_KMPH)
    globalWindDirectionDegrees = 90

    # Change wind counter
    changeWindCounter = 0
    changeWindMax = int(CHANGE_WIND_DIRECTION_PERIOD_SECONDS / WIND_PUBLISH_PERIOD_SECONDS)
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
        talker()
    except rospy.ROSInterruptException: pass
